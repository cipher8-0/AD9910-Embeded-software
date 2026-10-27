/**
 * @file    ad9910.h
 * @brief   AD9910 DDS driver for RaptorWatch (EEE4022S).
 *
 * The STM32 controls the AD9910 through two kinds of connection:
 *   1. SPI (SCLK, SDIO, SDO, CS) to read and write the AD9910's internal
 *      registers. This is how every setting (frequency, PLL, ramp) is loaded.
 *   2. Plain GPIO lines for real-time control: IO_UPDATE (apply buffered
 *      writes), MASTER_RESET, DRCTL (start a chirp), DRHOLD (freeze a chirp),
 *      and inputs DROVER / PLL_LOCK (status from the AD9910).
 *
 * Register addresses and bit positions below come from AD9910 Rev. E,
 * Table 17 (p.49) and the bit description tables after it.
 * See AD9910_DRIVER_EXPLAINER.md for the full walkthrough.
 */
#ifndef AD9910_H
#define AD9910_H

#include <stdint.h>
#include <stdbool.h>

/* CubeMX generates main.h for every project and it includes the correct HAL
 * for the chip (stm32l4xx_hal.h on the L476, stm32f4xx_hal.h on the F446).
 * Including it here means this driver builds on either board unchanged. */
#include "main.h"

/* ======================================================================
 * Register addresses (5-bit serial address, Table 17)
 * ====================================================================== */
#define AD9910_REG_CFR1         0x00u   /* control function register 1 */
#define AD9910_REG_CFR2         0x01u   /* control function register 2 */
#define AD9910_REG_CFR3         0x02u   /* control function register 3 (clock/PLL) */
#define AD9910_REG_AUX_DAC      0x03u   /* DAC full-scale current */
#define AD9910_REG_IO_UPD_RATE  0x04u
#define AD9910_REG_FTW          0x07u   /* frequency tuning word */
#define AD9910_REG_POW          0x08u   /* phase offset word */
#define AD9910_REG_ASF          0x09u   /* amplitude scale factor */
#define AD9910_REG_MULTICHIP    0x0Au
#define AD9910_REG_DR_LIMIT     0x0Bu   /* ramp upper/lower limits */
#define AD9910_REG_DR_STEP      0x0Cu   /* ramp step sizes */
#define AD9910_REG_DR_RATE      0x0Du   /* ramp step intervals */
#define AD9910_REG_PROFILE0     0x0Eu   /* single tone profiles 0..7 */
#define AD9910_REG_PROFILE7     0x15u

/* ======================================================================
 * CFR1 bits
 * ====================================================================== */
#define AD9910_CFR1_RAM_ENABLE        (1UL << 31)
#define AD9910_CFR1_INV_SINC_EN       (1UL << 22)
#define AD9910_CFR1_SELECT_SINE       (1UL << 16)   /* 0 = cosine output, 1 = sine */
#define AD9910_CFR1_LOAD_LRR          (1UL << 15)   /* reload ramp timer on IO_UPDATE */
#define AD9910_CFR1_AUTOCLR_DRG       (1UL << 14)   /* clear ramp accumulator for 1 cycle on IO_UPDATE */
#define AD9910_CFR1_AUTOCLR_PHASE     (1UL << 13)
#define AD9910_CFR1_CLR_DRG           (1UL << 12)   /* hold ramp accumulator in clear */
#define AD9910_CFR1_CLR_PHASE         (1UL << 11)
#define AD9910_CFR1_OSK_ENABLE        (1UL << 9)
#define AD9910_CFR1_DIGITAL_PD        (1UL << 7)
#define AD9910_CFR1_DAC_PD            (1UL << 6)
#define AD9910_CFR1_REFCLK_PD         (1UL << 5)
#define AD9910_CFR1_SDIO_INPUT_ONLY   (1UL << 1)    /* 4-wire SPI: read data comes out on SDO */
#define AD9910_CFR1_LSB_FIRST         (1UL << 0)

/* ======================================================================
 * CFR2 bits
 * ====================================================================== */
#define AD9910_CFR2_DEFAULT           0x00400820UL  /* value after MASTER_RESET */

/* Bits that actually exist in CFR2. "Open" (unused) bits are excluded so a
 * readback comparison is not thrown off by whatever the chip returns there. */
#define AD9910_CFR2_DEFINED_MASK      0x01FFCEFFUL

#define AD9910_CFR2_EN_ASF_PROFILE    (1UL << 24)   /* take amplitude from the active profile */
#define AD9910_CFR2_INT_IO_UPDATE     (1UL << 23)
#define AD9910_CFR2_SYNC_CLK_EN       (1UL << 22)   /* drives the SYNC_CLK pin at f_SYSCLK/4 */
#define AD9910_CFR2_DRG_DEST(x)       (((uint32_t)(x) & 0x3u) << 20)   /* bits 21:20 */
#define AD9910_CFR2_DRG_ENABLE        (1UL << 19)
#define AD9910_CFR2_DRG_NODWELL_HIGH  (1UL << 18)
#define AD9910_CFR2_DRG_NODWELL_LOW   (1UL << 17)
#define AD9910_CFR2_READ_EFF_FTW      (1UL << 16)
#define AD9910_CFR2_PDCLK_EN          (1UL << 11)   /* parallel port clock output */
#define AD9910_CFR2_SYNC_TIMING_DIS   (1UL << 5)
#define AD9910_CFR2_PARALLEL_EN       (1UL << 4)

#define AD9910_DRG_DEST_FREQ          0u   /* ramp drives frequency (chirp) */
#define AD9910_DRG_DEST_PHASE         1u
#define AD9910_DRG_DEST_AMPL          2u

/* ======================================================================
 * CFR3 bits and fields (reference clock and PLL)
 * ====================================================================== */
#define AD9910_CFR3_DEFAULT           0x1F3F4000UL  /* value after MASTER_RESET */
#define AD9910_CFR3_DEFINED_MASK      0x3738C5FEUL  /* excludes open bits 27, 23:22, 18:16, 13:11, 9, 0 */

#define AD9910_CFR3_DRV0(x)           (((uint32_t)(x) & 0x3u) << 28)   /* REFCLK_OUT drive, 0 = off */
#define AD9910_CFR3_VCO_SEL(x)        (((uint32_t)(x) & 0x7u) << 24)   /* VCO band, 110/111 = PLL bypassed */
#define AD9910_CFR3_ICP(x)            (((uint32_t)(x) & 0x7u) << 19)   /* charge pump current, 212..387 uA */
#define AD9910_CFR3_REFCLK_DIV_BYPASS (1UL << 15)   /* 1 = skip the /2 input divider */
#define AD9910_CFR3_REFCLK_DIV_RESETB (1UL << 14)   /* 1 = divider running (not held in reset) */
#define AD9910_CFR3_PFD_RESET         (1UL << 10)   /* 1 = phase detector disabled, clears PLL_LOCK */
#define AD9910_CFR3_PLL_ENABLE        (1UL << 8)
#define AD9910_CFR3_N(x)              (((uint32_t)(x) & 0x7Fu) << 1)   /* PLL multiplier, bits 7:1 */

#define AD9910_VCO5                   5u
#define AD9910_VCO_BYPASS             7u

/* PLL limits from the datasheet */
#define AD9910_PLL_N_MIN              12u
#define AD9910_PLL_N_MAX              127u
#define AD9910_FSYSCLK_PLL_MIN_HZ     420000000ULL
#define AD9910_FSYSCLK_MAX_HZ         1000000000ULL

/* ======================================================================
 * Driver tuning constant
 * ====================================================================== */
/* Smallest ramp step (in FTW LSBs) the chirp planner will use. Each step is
 * rounded to an integer, so the slope error is about 0.5/STEP. 1000 keeps the
 * slope error near 0.05%. See the explainer, section 7. */
#define AD9910_DRG_MIN_STEP           1000.0

/* ======================================================================
 * Types
 * ====================================================================== */
typedef enum {
    AD9910_OK = 0,
    AD9910_ERR_SPI,        /* HAL SPI call failed or timed out */
    AD9910_ERR_PARAM,      /* argument out of range */
    AD9910_ERR_PLL,        /* PLL_LOCK did not go high in time */
    AD9910_ERR_COMMS,      /* SPI worked but readback did not match */
    AD9910_ERR_NOT_INIT    /* called before a successful ad9910_init() */
} ad9910_status_t;

typedef enum {
    AD9910_ARM_A_XTAL_PLL,   /* onboard 25 MHz crystal -> internal PLL */
    AD9910_ARM_B_EXT_PLL,    /* external 25 MHz        -> internal PLL */
    AD9910_ARM_C_DIRECT      /* ~1 GHz direct, PLL bypassed          */
} ad9910_clk_arm_t;

typedef enum {
    AD9910_CHIRP_TRIGGERED_SAW,  /* one up-ramp per DRCTL rising edge, then snaps back */
    AD9910_CHIRP_TRIANGLE_FREE   /* free-running up/down ramp, DRCTL ignored */
} ad9910_chirp_mode_t;

/* One STM32 pin. 'port' is the GPIO block (GPIOA, GPIOB, ...), which is a
 * pointer to that block's registers in memory. 'pin' is a bit mask such as
 * GPIO_PIN_5. Set port = NULL for any optional line that is not wired. */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} ad9910_pin_t;

/* Everything the driver needs to know about one AD9910. Fill the "user"
 * fields in main.c, then call ad9910_init(). */
typedef struct {
    /* --- user: hardware bindings --------------------------------------- */
    SPI_HandleTypeDef *hspi;       /* e.g. &hspi1, created by CubeMX */
    ad9910_pin_t cs;               /* required */
    ad9910_pin_t io_update;        /* required */
    ad9910_pin_t master_reset;     /* required */
    ad9910_pin_t drctl;            /* required for chirps */
    ad9910_pin_t io_reset;         /* optional (port = NULL if not wired) */
    ad9910_pin_t drhold;           /* optional */
    ad9910_pin_t drover;           /* optional input */
    ad9910_pin_t pll_lock;         /* optional input; blind wait used if absent */

    /* --- user: clock configuration ------------------------------------- */
    ad9910_clk_arm_t arm;
    uint32_t refclk_hz;            /* 25e6 for arms A/B, ~1e9 for arm C */
    uint8_t  pll_n;                /* 40 for 25 MHz -> 1 GHz (arms A/B only) */
    uint8_t  icp_code;             /* 0..7, charge pump code the loop filter was designed for */

    /* --- driver: filled in by ad9910_init() ---------------------------- */
    uint64_t fsysclk_hz;           /* 0 until init succeeds */
    uint32_t cfr1_shadow, cfr2_shadow, cfr3_shadow;
} ad9910_t;

typedef struct {
    uint64_t            f_start_hz;
    uint64_t            f_stop_hz;
    double              t_chirp_s;
    ad9910_chirp_mode_t mode;
} ad9910_chirp_cfg_t;

/* What the chirp planner actually programmed, after integer rounding */
typedef struct {
    uint32_t ftw_lo, ftw_hi, step;
    uint16_t rate_p;
    double   t_chirp_actual_s;
    double   slope_hz_per_s;
} ad9910_chirp_result_t;

/* ======================================================================
 * API
 * ====================================================================== */
/* Low level */
uint8_t         ad9910_reg_len(uint8_t addr);
ad9910_status_t ad9910_write_reg(ad9910_t *dev, uint8_t addr, uint64_t value);
ad9910_status_t ad9910_read_reg(ad9910_t *dev, uint8_t addr, uint64_t *value);
void            ad9910_io_update(ad9910_t *dev);
void            ad9910_master_reset(ad9910_t *dev);

/* Unit conversion */
uint32_t        ad9910_freq_to_ftw(const ad9910_t *dev, uint64_t f_hz);
double          ad9910_ftw_to_freq(const ad9910_t *dev, uint32_t ftw);

/* High level */
ad9910_status_t ad9910_init(ad9910_t *dev);
ad9910_status_t ad9910_set_single_tone(ad9910_t *dev, uint64_t f_hz, uint16_t asf, uint16_t pow);
ad9910_status_t ad9910_configure_chirp(ad9910_t *dev, const ad9910_chirp_cfg_t *cfg,
                                       ad9910_chirp_result_t *res);
void            ad9910_chirp_trigger(ad9910_t *dev);
void            ad9910_drg_hold(ad9910_t *dev, bool hold);
bool            ad9910_drover(ad9910_t *dev);

#endif /* AD9910_H */
