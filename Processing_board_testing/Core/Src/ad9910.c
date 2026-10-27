/**
 * @file    ad9910.c
 * @brief   AD9910 DDS driver for RaptorWatch (EEE4022S).
 *
 * Layers in this file, bottom to top:
 *   1. GPIO helpers        : drive/read single STM32 pins
 *   2. SPI register access : one AD9910 register per SPI transaction
 *   3. Control pulses      : IO_UPDATE and MASTER_RESET
 *   4. Unit conversion     : Hz <-> 32-bit frequency tuning word
 *   5. Init / tone / chirp : complete operations built from the layers above
 *
 * See AD9910_DRIVER_EXPLAINER.md for the theory behind each layer.
 */
#include "ad9910.h"
#include <math.h>     /* ceil, llround (chirp planner) */
#include <stddef.h>   /* NULL */

/* HAL SPI functions take a timeout in ms, measured with the 1 ms SysTick.
 * A full 9-byte frame at 5 MHz takes about 15 us, so 10 ms means "broken". */
#define AD9910_SPI_TIMEOUT_MS       10u
#define AD9910_PLL_LOCK_TIMEOUT_MS  10u

/* ======================================================================
 * 1. GPIO helpers
 * ======================================================================
 * HAL_GPIO_WritePin() writes the pin's bit into the port's BSRR register
 * (bit set/reset register). BSRR changes one pin atomically without a
 * read-modify-write, so an interrupt cannot corrupt other pins on the port.
 * HAL_GPIO_ReadPin() reads the port's IDR (input data register).
 *
 * Both helpers do nothing for pins left unbound (port == NULL), so optional
 * lines such as DRHOLD can simply be omitted in main.c.
 */
static inline void pin_write(const ad9910_pin_t *p, GPIO_PinState s)
{
    if (p->port != NULL) {
        HAL_GPIO_WritePin(p->port, p->pin, s);
    }
}

static inline GPIO_PinState pin_read(const ad9910_pin_t *p)
{
    return (p->port != NULL) ? HAL_GPIO_ReadPin(p->port, p->pin) : GPIO_PIN_RESET;
}

/* CS is active low: pulling it low tells the AD9910 a new transaction is
 * starting. The first byte it then receives is always an instruction byte.
 * CS is a normal GPIO here (software NSS), not the SPI peripheral's NSS pin,
 * so we control exactly when it goes low and high. */
static inline void cs_low(ad9910_t *dev)  { pin_write(&dev->cs, GPIO_PIN_RESET); }
static inline void cs_high(ad9910_t *dev) { pin_write(&dev->cs, GPIO_PIN_SET);   }

/* Busy-wait of roughly 1 to 2 us, used to stretch control pulses.
 * The AD9910 samples IO_UPDATE with its internal SYNC_CLK (= f_SYSCLK / 4),
 * and the pulse must stay high for more than one SYNC_CLK period. Right after
 * reset, before the PLL is on, f_SYSCLK can be as low as 12.5 MHz, giving a
 * 320 ns period, so a microsecond is a safe margin.
 * SystemCoreClock is the CPU clock in Hz (80e6 on the L476 setup), maintained
 * by the HAL. Each loop pass costs at least ~4 CPU cycles, so dividing by
 * 4e6 gives at least ~1 us. 'volatile' stops the compiler removing the loop. */
static void short_delay(void)
{
    uint32_t n = SystemCoreClock / 4000000u;
    for (volatile uint32_t i = 0; i < n; i++) { __NOP(); }
}

/* ======================================================================
 * 2. SPI register access
 * ====================================================================== */

/* Thin wrappers around the HAL SPI calls. HAL_SPI_MODULE_ENABLED is defined
 * in stm32l4xx_hal_conf.h only when an SPI peripheral is enabled in CubeMX.
 * Before that (dry-run builds), the driver still compiles and these simply
 * report an error, so nothing ever touches hardware that is not set up. */
static HAL_StatusTypeDef spi_tx(ad9910_t *dev, uint8_t *data, uint16_t n)
{
#ifdef HAL_SPI_MODULE_ENABLED
    return HAL_SPI_Transmit(dev->hspi, data, n, AD9910_SPI_TIMEOUT_MS);
#else
    (void)dev; (void)data; (void)n;
    return HAL_ERROR;
#endif
}

static HAL_StatusTypeDef spi_rx(ad9910_t *dev, uint8_t *data, uint16_t n)
{
#ifdef HAL_SPI_MODULE_ENABLED
    return HAL_SPI_Receive(dev->hspi, data, n, AD9910_SPI_TIMEOUT_MS);
#else
    (void)dev; (void)data; (void)n;
    return HAL_ERROR;
#endif
}

/* Register size in bytes. The SPI frame length depends on it, and the
 * AD9910 expects exactly this many data bytes after the instruction byte. */
uint8_t ad9910_reg_len(uint8_t addr)
{
    switch (addr) {
    case AD9910_REG_CFR1:
    case AD9910_REG_CFR2:
    case AD9910_REG_CFR3:
    case AD9910_REG_AUX_DAC:
    case AD9910_REG_IO_UPD_RATE:
    case AD9910_REG_FTW:
    case AD9910_REG_ASF:
    case AD9910_REG_MULTICHIP:
    case AD9910_REG_DR_RATE:
        return 4;
    case AD9910_REG_POW:
        return 2;
    case AD9910_REG_DR_LIMIT:
    case AD9910_REG_DR_STEP:
        return 8;
    default:
        if (addr >= AD9910_REG_PROFILE0 && addr <= AD9910_REG_PROFILE7) {
            return 8;
        }
        return 0;   /* open addresses 0x05/0x06, and RAM (0x16) is not supported */
    }
}

/* Write one register.
 * SPI frame (CS low throughout):
 *   byte 0      : instruction = [R/W=0][x][x][A4..A0]
 *   bytes 1..len: data, most significant byte first
 * The data goes into the AD9910's I/O buffer, NOT the active register.
 * It only takes effect at the next IO_UPDATE pulse. */
ad9910_status_t ad9910_write_reg(ad9910_t *dev, uint8_t addr, uint64_t value)
{
    uint8_t len = ad9910_reg_len(addr);
    if (len == 0) {
        return AD9910_ERR_PARAM;
    }

    uint8_t buf[1 + 8];   /* instruction + largest register */

    /* Bit 7 = 0 means write. Masking with 0x1F keeps a bad address from
     * spilling into bit 7 and accidentally turning this into a read. */
    buf[0] = (uint8_t)(addr & 0x1Fu);

    /* Split the value into bytes, MSB first: for a 4-byte register,
     * i = 0 takes bits 31:24 (shift 24), i = 3 takes bits 7:0 (shift 0). */
    for (uint8_t i = 0; i < len; i++) {
        buf[1 + i] = (uint8_t)(value >> (8u * (len - 1u - i)));
    }

    /* HAL_SPI_Transmit (via spi_tx) is a blocking (polled) transfer: the
     * CPU feeds each byte into the SPI data register (SPIx->DR) and waits
     * for the TXE flag before the next one. The SPI hardware shifts the
     * bits out on MOSI (to AD9910 SDIO) while generating SCLK. */
    cs_low(dev);
    HAL_StatusTypeDef st = spi_tx(dev, buf, (uint16_t)(1u + len));
    cs_high(dev);

    return (st == HAL_OK) ? AD9910_OK : AD9910_ERR_SPI;
}

/* Read one register.
 * Requires 4-wire mode (CFR1 SDIO_INPUT_ONLY = 1), so the AD9910 sends its
 * reply on the SDO pin, which is wired to the STM32's MISO. */
ad9910_status_t ad9910_read_reg(ad9910_t *dev, uint8_t addr, uint64_t *value)
{
    uint8_t len = ad9910_reg_len(addr);
    if (len == 0 || value == NULL) {
        return AD9910_ERR_PARAM;
    }

    uint8_t instr = (uint8_t)(0x80u | (addr & 0x1Fu));   /* bit 7 = 1 means read */
    uint8_t rx[8] = {0};

    /* CS stays low across both calls: the AD9910 sees one continuous frame.
     * SPI is full duplex, so to receive, the master must still generate
     * clocks. HAL_SPI_Receive does this by transmitting dummy bytes while
     * collecting whatever arrives on MISO. */
    cs_low(dev);
    HAL_StatusTypeDef st = spi_tx(dev, &instr, 1);
    if (st == HAL_OK) {
        st = spi_rx(dev, rx, len);
    }
    cs_high(dev);

    if (st != HAL_OK) {
        return AD9910_ERR_SPI;
    }

    /* Reassemble MSB first: each new byte pushes the previous ones up. */
    uint64_t v = 0;
    for (uint8_t i = 0; i < len; i++) {
        v = (v << 8) | rx[i];
    }
    *value = v;
    return AD9910_OK;
}

/* ======================================================================
 * 3. Control pulses
 * ====================================================================== */

/* IO_UPDATE: copies everything written since the last update from the I/O
 * buffer into the active registers, all at the same instant. This is what
 * lets several registers (limits, step, rate, CFR2) change together. */
void ad9910_io_update(ad9910_t *dev)
{
    pin_write(&dev->io_update, GPIO_PIN_SET);
    short_delay();
    pin_write(&dev->io_update, GPIO_PIN_RESET);
}

/* MASTER_RESET (active high) returns every register to its default value.
 * HAL_Delay(1) blocks on the SysTick counter for at least 1 ms, far more
 * than the reset pulse the chip needs. */
void ad9910_master_reset(ad9910_t *dev)
{
    pin_write(&dev->master_reset, GPIO_PIN_SET);
    HAL_Delay(1);
    pin_write(&dev->master_reset, GPIO_PIN_RESET);
    HAL_Delay(1);

    /* The shadows are RAM copies of what we believe is in CFR1..3. The chip
     * is at defaults now, so the copies must match that. */
    dev->cfr1_shadow = 0x00000000u;
    dev->cfr2_shadow = AD9910_CFR2_DEFAULT;
    dev->cfr3_shadow = AD9910_CFR3_DEFAULT;
}

/* ======================================================================
 * 4. Frequency <-> tuning word
 *    FTW = round(2^32 * f_out / f_SYSCLK)
 * ======================================================================
 * Integer maths only. The Cortex-M4 FPU is single precision (24-bit
 * mantissa), too coarse for a 32-bit FTW. 64-bit integers are exact:
 * f << 32 fits in 64 bits for any f below 4.29 GHz. Adding half the divisor
 * before dividing rounds to nearest instead of truncating. */
uint32_t ad9910_freq_to_ftw(const ad9910_t *dev, uint64_t f_hz)
{
    if (dev->fsysclk_hz == 0) {
        return 0;   /* not initialised; avoids a divide by zero */
    }
    uint64_t num = (f_hz << 32) + (dev->fsysclk_hz / 2u);
    return (uint32_t)(num / dev->fsysclk_hz);
}

/* Inverse, for reporting the frequency actually produced. double is
 * emulated in software on the M4 (slower, but exact enough and rarely used). */
double ad9910_ftw_to_freq(const ad9910_t *dev, uint32_t ftw)
{
    return (double)ftw * (double)dev->fsysclk_hz / 4294967296.0;
}

/* ======================================================================
 * 5. Initialisation
 * ====================================================================== */

/* Waits for the AD9910's PLL_LOCK output (active high) to go high.
 * HAL_GetTick() returns ms since boot (SysTick interrupt count); comparing
 * with a start time gives a timeout without blocking forever. */
static ad9910_status_t wait_pll_lock(ad9910_t *dev)
{
    if (dev->pll_lock.port == NULL) {
        HAL_Delay(5);   /* no lock pin wired: wait far longer than lock takes */
        return AD9910_OK;
    }
    uint32_t t0 = HAL_GetTick();
    while (pin_read(&dev->pll_lock) != GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > AD9910_PLL_LOCK_TIMEOUT_MS) {
            return AD9910_ERR_PLL;
        }
    }
    return AD9910_OK;
}

/* Read a 32-bit register and compare its defined bits with 'expected'.
 * On a mismatch, record what was seen so the caller can report it. */
static ad9910_status_t check_reg(ad9910_t *dev, uint8_t addr, uint32_t expected, uint32_t mask)
{
    uint64_t readback = 0;
    ad9910_status_t st = ad9910_read_reg(dev, addr, &readback);
    if (st != AD9910_OK) return st;
    if (((uint32_t)readback & mask) != (expected & mask)) {
        dev->diag_addr     = addr;
        dev->diag_read     = (uint32_t)readback;
        dev->diag_expected = expected & mask;
        return AD9910_ERR_COMMS;
    }
    return AD9910_OK;
}

/* Write CFR3 from its shadow and apply it. */
static ad9910_status_t write_cfr3(ad9910_t *dev)
{
    ad9910_status_t st = ad9910_write_reg(dev, AD9910_REG_CFR3, dev->cfr3_shadow);
    if (st == AD9910_OK) {
        ad9910_io_update(dev);
    }
    return st;
}

ad9910_status_t ad9910_init(ad9910_t *dev)
{
    ad9910_status_t st;

    dev->fsysclk_hz = 0;   /* marks "not initialised" until we succeed */
    dev->diag_addr = 0;
    dev->diag_read = dev->diag_expected = 0;

    if (dev->hspi == NULL || dev->cs.port == NULL ||
        dev->io_update.port == NULL || dev->master_reset.port == NULL) {
        return AD9910_ERR_PARAM;
    }

#ifdef HAL_SPI_MODULE_ENABLED
    /* The HAL only sets SPE (SPI enable) inside the first transfer, i.e.
     * after CS has already gone low. Until then the L4 SPI does not drive
     * SCK, so the pin floats (no pull in CubeMX) and the AD9910 could see a
     * stray edge at the start of the first frame. Enable the SPI now, while
     * CS is still high, so SCK sits at its CPOL idle level (low). */
    __HAL_SPI_ENABLE(dev->hspi);
#endif

    /* 1. Idle levels on all outputs before touching the chip:
     *    CS high (no transaction), pulses low. */
    cs_high(dev);
    pin_write(&dev->io_update, GPIO_PIN_RESET);
    pin_write(&dev->io_reset,  GPIO_PIN_RESET);
    pin_write(&dev->drctl,     GPIO_PIN_RESET);
    pin_write(&dev->drhold,    GPIO_PIN_RESET);

    /* 2. Hard reset: known register state regardless of what ran before. */
    ad9910_master_reset(dev);

    /* 3. Switch the serial port to 4-wire mode so reads come back on SDO.
     *    After reset the chip is in 3-wire mode (SDIO is bidirectional). */
    dev->cfr1_shadow = AD9910_CFR1_SDIO_INPUT_ONLY;
    st = ad9910_write_reg(dev, AD9910_REG_CFR1, dev->cfr1_shadow);
    if (st != AD9910_OK) return st;
    ad9910_io_update(dev);

    /* 4. Comms smoke test: CFR3 has not been written since reset, so it must
     *    read back as its default. Only defined bits are compared. A failure
     *    here means SPI wiring, SPI mode, or SDO/MISO is wrong. */
    st = check_reg(dev, AD9910_REG_CFR3, AD9910_CFR3_DEFAULT, AD9910_CFR3_DEFINED_MASK);
    if (st != AD9910_OK) return st;

    /* 5. Clock configuration for the selected GA4 arm.
     *    Starting from zero also turns REFCLK_OUT off (DRV0 = 00). The input
     *    /2 divider is bypassed in every arm: arms A/B feed 25 MHz straight
     *    to the PFD (the loop filter was designed for a 25 MHz PFD), and arm C
     *    feeds ~1 GHz straight to the core. */
    uint64_t fsys;
    uint32_t cfr3 = AD9910_CFR3_REFCLK_DIV_RESETB | AD9910_CFR3_REFCLK_DIV_BYPASS;

    switch (dev->arm) {
    case AD9910_ARM_A_XTAL_PLL:
    case AD9910_ARM_B_EXT_PLL:
        /* A and B differ only in hardware (crystal vs external source);
         * the register settings are identical. */
        if (dev->pll_n < AD9910_PLL_N_MIN || dev->pll_n > AD9910_PLL_N_MAX) {
            return AD9910_ERR_PARAM;
        }
        fsys = (uint64_t)dev->refclk_hz * dev->pll_n;
        if (fsys < AD9910_FSYSCLK_PLL_MIN_HZ || fsys > AD9910_FSYSCLK_MAX_HZ) {
            return AD9910_ERR_PARAM;
        }
        cfr3 |= AD9910_CFR3_VCO_SEL(AD9910_VCO5)
              | AD9910_CFR3_ICP(dev->icp_code)
              | AD9910_CFR3_PLL_ENABLE
              | AD9910_CFR3_N(dev->pll_n);
        break;

    case AD9910_ARM_C_DIRECT:
        if (dev->refclk_hz == 0 || dev->refclk_hz > AD9910_FSYSCLK_MAX_HZ) {
            return AD9910_ERR_PARAM;
        }
        fsys = dev->refclk_hz;
        cfr3 |= AD9910_CFR3_VCO_SEL(AD9910_VCO_BYPASS);   /* PLL off */
        break;

    default:
        return AD9910_ERR_PARAM;
    }

    if (dev->arm != AD9910_ARM_C_DIRECT) {
        /* 6. PLL_LOCK is a LATCHED output and may already read 1 from the
         *    bypassed state after reset. Pulse PFD reset first to clear it,
         *    so the wait below sees a genuine lock, not a stale one. */
        dev->cfr3_shadow = cfr3 | AD9910_CFR3_PFD_RESET;
        if ((st = write_cfr3(dev)) != AD9910_OK) return st;
    }

    dev->cfr3_shadow = cfr3;
    if ((st = write_cfr3(dev)) != AD9910_OK) return st;

    if (dev->arm != AD9910_ARM_C_DIRECT) {
        st = wait_pll_lock(dev);
        if (st != AD9910_OK) return st;
    }

    /* 7. Base CFR2: defaults with the PDCLK output switched off, since the
     *    parallel port is unused and a toggling clock pin only adds spurs. */
    dev->cfr2_shadow = AD9910_CFR2_DEFAULT & ~AD9910_CFR2_PDCLK_EN;
    st = ad9910_write_reg(dev, AD9910_REG_CFR2, dev->cfr2_shadow);
    if (st != AD9910_OK) return st;
    ad9910_io_update(dev);

    /* 8. Read CFR2 back to prove writes land, now at the final clock rate. */
    st = check_reg(dev, AD9910_REG_CFR2, dev->cfr2_shadow, AD9910_CFR2_DEFINED_MASK);
    if (st != AD9910_OK) return st;

    dev->fsysclk_hz = fsys;   /* only set once everything has succeeded */
    return AD9910_OK;
}

/* ======================================================================
 * Single tone: fixed frequency, amplitude, phase from profile 0.
 * The PROFILE[2:0] pins select which of the 8 profiles is active, so they
 * must be 000 (tied low or driven low) for this to take effect.
 * ====================================================================== */
ad9910_status_t ad9910_set_single_tone(ad9910_t *dev, uint64_t f_hz, uint16_t asf, uint16_t pow)
{
    if (dev->fsysclk_hz == 0) return AD9910_ERR_NOT_INIT;
    if (asf > 0x3FFFu) return AD9910_ERR_PARAM;                         /* ASF is 14 bits */
    if (f_hz > (dev->fsysclk_hz * 2u) / 5u) return AD9910_ERR_PARAM;    /* stay below 0.4 f_SYSCLK */

    uint32_t ftw = ad9910_freq_to_ftw(dev, f_hz);

    /* DRG off (it would override the profile frequency) and amplitude taken
     * from the profile's ASF field. */
    dev->cfr2_shadow = (dev->cfr2_shadow & ~AD9910_CFR2_DRG_ENABLE) | AD9910_CFR2_EN_ASF_PROFILE;

    /* Profile register layout: [63:62] open, [61:48] ASF, [47:32] POW, [31:0] FTW.
     * Cast to 64 bits BEFORE shifting: a 16-bit value shifted by 48 in a
     * 32-bit int would lose all its bits. */
    uint64_t prof = ((uint64_t)(asf & 0x3FFFu) << 48)
                  | ((uint64_t)pow << 32)
                  | (uint64_t)ftw;

    ad9910_status_t st = ad9910_write_reg(dev, AD9910_REG_CFR2, dev->cfr2_shadow);
    if (st != AD9910_OK) return st;
    st = ad9910_write_reg(dev, AD9910_REG_PROFILE0, prof);
    if (st != AD9910_OK) return st;
    ad9910_io_update(dev);   /* both writes take effect together */
    return AD9910_OK;
}

/* ======================================================================
 * Linear frequency chirp using the Digital Ramp Generator (DRG)
 *
 *   step interval   dt = 4 * P / f_SYSCLK     (DRG clock is f_SYSCLK / 4)
 *   freq per step   df = STEP * f_SYSCLK / 2^32
 *   chirp time      T  = (span / STEP) * dt,   span = FTW_hi - FTW_lo
 *
 * The planner picks P and STEP so the ramp matches the requested T while
 * keeping STEP >= AD9910_DRG_MIN_STEP. Worked example in the explainer.
 * ====================================================================== */

/* Planner only: computes P, STEP and the achieved chirp without touching
 * the hardware. Split out so the command layer can report a plan (for
 * example in dry-run mode, or to validate GUI input) before committing. */
ad9910_status_t ad9910_plan_chirp(const ad9910_t *dev, const ad9910_chirp_cfg_t *cfg,
                                  ad9910_chirp_result_t *res)
{
    if (dev->fsysclk_hz == 0) return AD9910_ERR_NOT_INIT;
    if (cfg == NULL || res == NULL) return AD9910_ERR_PARAM;
    if (cfg->f_start_hz >= cfg->f_stop_hz) return AD9910_ERR_PARAM;
    if (cfg->f_stop_hz > (dev->fsysclk_hz * 2u) / 5u) return AD9910_ERR_PARAM;
    if (cfg->t_chirp_s <= 0.0) return AD9910_ERR_PARAM;

    const double fs = (double)dev->fsysclk_hz;
    const double T  = cfg->t_chirp_s;

    uint32_t ftw_lo = ad9910_freq_to_ftw(dev, cfg->f_start_hz);
    uint32_t ftw_hi = ad9910_freq_to_ftw(dev, cfg->f_stop_hz);
    uint32_t span   = ftw_hi - ftw_lo;
    if (span == 0) return AD9910_ERR_PARAM;

    /* Rate word P: smallest step interval that still allows STEP >= MIN_STEP.
     * From T = (span / STEP) * 4P / fs with STEP = MIN_STEP. Rounded UP,
     * because a smaller P would force STEP below the minimum. */
    double p_d = ceil(AD9910_DRG_MIN_STEP * fs * T / (4.0 * (double)span));
    if (p_d < 1.0) p_d = 1.0;
    if (p_d > 65535.0) return AD9910_ERR_PARAM;   /* too slow for the 16-bit timer */
    uint16_t P = (uint16_t)p_d;

    /* STEP for that P, rounded to nearest. Range-checked as a double first:
     * casting a value above 2^32 to uint32_t would silently wrap. */
    double step_d = (double)span * 4.0 * (double)P / (fs * T);
    if (step_d < 0.5 || step_d > (double)span) return AD9910_ERR_PARAM;   /* too slow / too fast */
    uint32_t step = (uint32_t)llround(step_d);

    /* What the hardware will actually do after rounding */
    uint32_t n_steps = (span + step - 1u) / step;   /* integer ceil(span / step) */
    res->t_chirp_actual_s = (double)n_steps * 4.0 * (double)P / fs;
    res->slope_hz_per_s   = ((double)step * fs / 4294967296.0) / (4.0 * (double)P / fs);
    res->ftw_lo  = ftw_lo;
    res->ftw_hi  = ftw_hi;
    res->step    = step;
    res->rate_p  = P;
    res->n_steps = n_steps;
    return AD9910_OK;
}

/* Plan the chirp, then load it into the DRG. */
ad9910_status_t ad9910_configure_chirp(ad9910_t *dev, const ad9910_chirp_cfg_t *cfg,
                                       ad9910_chirp_result_t *res)
{
    ad9910_status_t st = ad9910_plan_chirp(dev, cfg, res);
    if (st != AD9910_OK) return st;

    /* Pack the three DRG registers */
    uint64_t limit = ((uint64_t)res->ftw_hi << 32) | res->ftw_lo;  /* [63:32] upper, [31:0] lower */
    uint64_t steps = ((uint64_t)res->step   << 32) | res->step;    /* [63:32] down,  [31:0] up    */
    uint32_t rate  = ((uint32_t)res->rate_p << 16) | res->rate_p;  /* [31:16] down,  [15:0] up    */

    /* CFR2: clear every DRG field and the profile-amplitude bit, then enable
     * the DRG with frequency as its destination. With the amplitude bit clear
     * and OSK off, the output runs at full-scale amplitude. */
    uint32_t cfr2 = dev->cfr2_shadow & ~(AD9910_CFR2_DRG_DEST(3u) | AD9910_CFR2_DRG_ENABLE |
                                         AD9910_CFR2_DRG_NODWELL_HIGH |
                                         AD9910_CFR2_DRG_NODWELL_LOW |
                                         AD9910_CFR2_EN_ASF_PROFILE);
    cfr2 |= AD9910_CFR2_DRG_DEST(AD9910_DRG_DEST_FREQ) | AD9910_CFR2_DRG_ENABLE;

    /* No-dwell bits choose the waveform:
     *   high only : rising DRCTL edge -> ramp up -> snap to lower limit, wait (sawtooth)
     *   both      : ramp up and down between the limits continuously (triangle) */
    if (cfg->mode == AD9910_CHIRP_TRIGGERED_SAW) {
        cfr2 |= AD9910_CFR2_DRG_NODWELL_HIGH;
    } else {
        cfr2 |= AD9910_CFR2_DRG_NODWELL_HIGH | AD9910_CFR2_DRG_NODWELL_LOW;
    }
    dev->cfr2_shadow = cfr2;

    /* With DRCTL = 0 when the DRG is enabled, the output starts at the lower
     * limit (f_start) and waits there for the first trigger. */
    pin_write(&dev->drctl, GPIO_PIN_RESET);

    if ((st = ad9910_write_reg(dev, AD9910_REG_DR_LIMIT, limit)) != AD9910_OK) return st;
    if ((st = ad9910_write_reg(dev, AD9910_REG_DR_STEP,  steps)) != AD9910_OK) return st;
    if ((st = ad9910_write_reg(dev, AD9910_REG_DR_RATE,  rate))  != AD9910_OK) return st;
    if ((st = ad9910_write_reg(dev, AD9910_REG_CFR2,     cfr2))  != AD9910_OK) return st;
    ad9910_io_update(dev);   /* all four registers take effect together */

    return AD9910_OK;
}

/* Start one sawtooth chirp. In no-dwell-high mode the AD9910 reacts only to
 * the rising EDGE of DRCTL, not its level, so we force low first to
 * guarantee an edge even if DRCTL was left high by the previous trigger. */
void ad9910_chirp_trigger(ad9910_t *dev)
{
    pin_write(&dev->drctl, GPIO_PIN_RESET);
    short_delay();
    pin_write(&dev->drctl, GPIO_PIN_SET);
}

/* DRHOLD high freezes the ramp at its current value; low resumes it. */
void ad9910_drg_hold(ad9910_t *dev, bool hold)
{
    pin_write(&dev->drhold, hold ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* DROVER is high whenever the ramp output sits at either limit. In sawtooth
 * mode it is high while waiting at f_start and low during the sweep, so
 * "low" means "chirp in progress". Useful as a scope trigger or, configured
 * as an EXTI interrupt input, to tell firmware exactly when a sweep ends. */
bool ad9910_drover(ad9910_t *dev)
{
    return pin_read(&dev->drover) == GPIO_PIN_SET;
}
