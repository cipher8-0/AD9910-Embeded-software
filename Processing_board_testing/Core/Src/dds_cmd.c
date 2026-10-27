/* ============================================================================
 * dds_cmd.c : turns GUI text commands into AD9910 driver calls
 * ============================================================================
 *
 * WHERE THIS RUNS
 * ---------------
 * DdsCmd_Handle() is called from UsbCmd_Poll() in the main while(1) loop,
 * never from an interrupt. That matters because some driver calls block:
 * ad9910_init() uses HAL_Delay() (a few ms) and every register access waits
 * for the SPI transfer to finish. Blocking is fine in the main loop; USB
 * reception carries on in the background because the USB interrupt keeps
 * filling the ring buffer in usb_cmd.c while we are busy here.
 *
 * WHAT IT DOES FOR EACH COMMAND
 * -----------------------------
 *   1. split the line into words ("CHIRP 100000000 200000000 100000 SAW")
 *   2. convert and range-check the numbers
 *   3. call the driver (or, in dry-run mode, only the driver's maths)
 *   4. build a one-line reply saying what was actually programmed
 *
 * It also keeps a small amount of STATE (dds_mode) so it can refuse
 * commands that make no sense right now, e.g. TRIG when no sawtooth chirp
 * is loaded.
 *
 * PRINTING NUMBERS ON AN MCU
 * --------------------------
 * CubeIDE links "newlib-nano", a size-reduced C library. Its printf/snprintf
 * leave out two things by default: floating point (%f) and 64-bit integers
 * (%llu). Using them silently prints nothing or garbage. So this file never
 * does either: doubles are rounded to integers first, and 64-bit values go
 * through u64_to_str() below. 32-bit values use %lu / %lX, which nano
 * supports (with an (unsigned long) cast, since uint32_t is 'unsigned long'
 * on some toolchains and 'unsigned int' on others).
 * ========================================================================== */

#include "dds_cmd.h"
#include "dds_board.h"
#include "ad9910.h"
#include "chirp_train.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#if !DDS_DRY_RUN
/* The SPI handle is defined by CubeMX in main.c. 'extern' lets us use it. */
extern SPI_HandleTypeDef DDS_SPI_HANDLE;
#endif

#define MAX_ARGS 6   /* longest command (CHIRP) has 5 words */

/* What the AD9910 is currently set up to do, as far as we know. */
typedef enum {
    MODE_NONE,      /* after reset/init, or not initialised */
    MODE_TONE,
    MODE_SAW,       /* triggered sawtooth chirp loaded, waiting for TRIG */
    MODE_TRI        /* free-running triangle chirp */
} dds_mode_t;

static ad9910_t   dds;                  /* driver state, one AD9910 */
static dds_mode_t dds_mode = MODE_NONE;

/* Achieved sweep time of the loaded sawtooth chirp, in ns. A chirp train
 * period must be longer than this (plus a margin), otherwise DRCTL edges
 * would arrive mid-sweep and those chirps would be skipped. */
static uint64_t   saw_sweep_ns = 0;
static ct_status_t train_init = CT_OK;   /* result of ChirpTrain_Init() */

/* ============================================================================
 * Small helpers
 * ========================================================================== */

/* uint64 -> decimal text, because newlib-nano cannot print %llu.
 * Builds digits backwards into a temp buffer, then copies them forwards. */
static const char *u64_to_str(uint64_t v, char *buf)
{
    char tmp[21];                 /* max 20 digits for 2^64-1, plus '\0' */
    int  i = 0;
    do {
        tmp[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v != 0u);
    int j = 0;
    while (i > 0) {
        buf[j++] = tmp[--i];
    }
    buf[j] = '\0';
    return buf;
}

/* Text -> uint64. Accepts decimal or 0x-prefixed hex (base 0 in strtoull).
 * Rejects empty strings, trailing junk ("100k") and negatives (strtoull
 * would quietly turn "-1" into 18446744073709551615). */
static int parse_u64(const char *s, uint64_t *out)
{
    if (s == NULL || *s == '\0' || *s == '-') return 0;
    char *end;
    unsigned long long v = strtoull(s, &end, 0);
    if (*end != '\0') return 0;
    *out = (uint64_t)v;
    return 1;
}

/* Driver status code -> short word for the reply line */
static const char *status_str(ad9910_status_t st)
{
    switch (st) {
    case AD9910_OK:           return "OK";
    case AD9910_ERR_SPI:      return "SPI";       /* HAL SPI timeout/error */
    case AD9910_ERR_PARAM:    return "PARAM";     /* value out of range */
    case AD9910_ERR_PLL:      return "PLL";       /* no lock within timeout */
    case AD9910_ERR_COMMS:    return "COMMS";     /* readback mismatch */
    case AD9910_ERR_NOT_INIT: return "NOT_INIT";  /* send DDS INIT first */
    default:                  return "UNKNOWN";
    }
}

static const char *mode_str(dds_mode_t m)
{
    switch (m) {
    case MODE_TONE: return "TONE";
    case MODE_SAW:  return "SAW";
    case MODE_TRI:  return "TRI";
    default:        return "NONE";
    }
}

/* Actual output frequency for a given FTW, as text with 3 decimals (mHz),
 * using integers only: f = FTW * f_SYSCLK / 2^32.
 * FTW < 2^32 and f_SYSCLK <= 1e9 < 2^30, so the product fits in 64 bits.
 * The top 32 bits of the product are whole Hz; the bottom 32 bits are the
 * fraction, which we scale to thousandths. */
static void ftw_to_hz_str(uint32_t ftw, char *buf, size_t n)
{
    uint64_t prod = (uint64_t)ftw * dds.fsysclk_hz;
    uint64_t hz   = prod >> 32;
    uint32_t mhz  = (uint32_t)(((prod & 0xFFFFFFFFull) * 1000u) >> 32);
    char h[21];
    snprintf(buf, n, "%s.%03lu", u64_to_str(hz, h), (unsigned long)mhz);
}

/* Picoseconds -> "ns.ppp" text, e.g. 200000000 ps -> "200000.000" */
static void ps_to_ns_str(uint64_t ps, char *buf, size_t n)
{
    char w[21];
    snprintf(buf, n, "%s.%03lu", u64_to_str(ps / 1000u, w), (unsigned long)(ps % 1000u));
}

static const char *ct_str(ct_status_t st)
{
    switch (st) {
    case CT_OK:         return "OK";
    case CT_ERR_PERIOD: return "PERIOD";   /* outside the timer's range */
    case CT_ERR_COUNT:  return "COUNT";    /* burst longer than RCR allows */
    case CT_ERR_CLOCK:  return "CLOCK";    /* TIM1 clock != DDS_TIM_CLK_HZ */
    case CT_ERR_HAL:    return "HAL";
    default:            return "UNKNOWN";
    }
}

static const char *dry_tag(void)
{
    return DDS_DRY_RUN ? " DRYRUN" : "";
}

/* ============================================================================
 * Setup
 * ========================================================================== */
void DdsCmd_Init(void)
{
    memset(&dds, 0, sizeof(dds));   /* all pins NULL, fsysclk_hz = 0 */

    dds.arm       = DDS_CLK_ARM;
    dds.refclk_hz = DDS_REFCLK_HZ;
    dds.pll_n     = DDS_PLL_N;
    dds.icp_code  = DDS_ICP_CODE;

#if !DDS_DRY_RUN
    /* Bind the driver to real STM32 hardware. Each port macro (GPIOA,
     * GPIOB, ...) is a pointer to that GPIO block's registers in memory,
     * e.g. GPIOB = 0x48000400 on the L476. The pins themselves must already
     * be configured as outputs/inputs by CubeMX's MX_GPIO_Init(). */
    dds.hspi         = &DDS_SPI_HANDLE;
    dds.cs           = (ad9910_pin_t){ DDS_CS_PORT,      DDS_CS_PIN };
    dds.io_update    = (ad9910_pin_t){ DDS_IOUPD_PORT,   DDS_IOUPD_PIN };
    dds.master_reset = (ad9910_pin_t){ DDS_MRESET_PORT,  DDS_MRESET_PIN };
    dds.drctl        = (ad9910_pin_t){ DDS_DRCTL_PORT,   DDS_DRCTL_PIN };
    dds.io_reset     = (ad9910_pin_t){ DDS_IORESET_PORT, DDS_IORESET_PIN };
    dds.drhold       = (ad9910_pin_t){ DDS_DRHOLD_PORT,  DDS_DRHOLD_PIN };
    dds.drover       = (ad9910_pin_t){ DDS_DROVER_PORT,  DDS_DROVER_PIN };
    dds.pll_lock     = (ad9910_pin_t){ DDS_PLLLOCK_PORT, DDS_PLLLOCK_PIN };
#endif

    dds_mode = MODE_NONE;
    saw_sweep_ns = 0;

    /* Configure TIM1 for chirp trains and check its clock. Must run after
     * MX_TIM1_Init(), which is why DdsCmd_Init() is called in USER CODE 2. */
    train_init = ChirpTrain_Init();
}

/* ============================================================================
 * Command handlers. Each writes exactly one reply line into 'out'.
 * ========================================================================== */

/* DDS INIT */
static void cmd_init(char *out, size_t n)
{
    char f[21];
    ChirpTrain_Stop();                        /* no triggers while reconfiguring */
#if DDS_DRY_RUN
    /* No chip: set f_SYSCLK directly so the tone/chirp maths can run.
     * This is the only place outside the driver that writes fsysclk_hz. */
    dds.fsysclk_hz = (dds.arm == AD9910_ARM_C_DIRECT)
                   ? (uint64_t)dds.refclk_hz
                   : (uint64_t)dds.refclk_hz * dds.pll_n;
#else
    ad9910_status_t st = ad9910_init(&dds);   /* blocks for a few ms */
    if (st == AD9910_ERR_COMMS) {
        /* Say which readback failed and what came back, so a wiring fault
         * (all 0s / all 1s) can be told apart from a bit-shifted frame. */
        dds_mode = MODE_NONE;
        snprintf(out, n, "ERR INIT COMMS REG 0x%02X READ 0x%08lX EXPECT 0x%08lX",
                 (unsigned)dds.diag_addr, (unsigned long)dds.diag_read,
                 (unsigned long)dds.diag_expected);
        return;
    }
    if (st != AD9910_OK) {
        dds_mode = MODE_NONE;
        snprintf(out, n, "ERR INIT %s", status_str(st));
        return;
    }
#endif
    dds_mode = MODE_NONE;
    snprintf(out, n, "OK INIT FSYS %s%s", u64_to_str(dds.fsysclk_hz, f), dry_tag());
}

/* DDS STATUS */
static void cmd_status(char *out, size_t n)
{
    char f[21];
    const char *lock = "NA";   /* not applicable / not wired */
#if !DDS_DRY_RUN
    if (dds.pll_lock.port != NULL) {
        /* Direct pin read through the GPIO input data register (IDR) */
        lock = (HAL_GPIO_ReadPin(dds.pll_lock.port, dds.pll_lock.pin) == GPIO_PIN_SET)
             ? "1" : "0";
    }
#endif
    snprintf(out, n, "OK STATUS INIT %d FSYS %s MODE %s LOCK %s TRAIN %s%s",
             dds.fsysclk_hz != 0, u64_to_str(dds.fsysclk_hz, f),
             mode_str(dds_mode), lock,
             ChirpTrain_IsRunning() ? "RUN" : "IDLE", dry_tag());
}

/* DDS RESET */
static void cmd_reset(char *out, size_t n)
{
    ChirpTrain_Stop();
#if !DDS_DRY_RUN
    ad9910_master_reset(&dds);
#endif
    /* After a reset the PLL is off and nothing is configured, so everything
     * that depends on f_SYSCLK must be refused until the next DDS INIT. */
    dds.fsysclk_hz = 0;
    dds_mode = MODE_NONE;
    saw_sweep_ns = 0;
    snprintf(out, n, "OK RESET%s", dry_tag());
}

/* TONE <f_hz> [asf] [pow] */
static void cmd_tone(int argc, char **argv, char *out, size_t n)
{
    uint64_t f, asf = 0x3FFF, pow = 0;

    if (argc < 2 || argc > 4 || !parse_u64(argv[1], &f) ||
        (argc >= 3 && !parse_u64(argv[2], &asf)) ||
        (argc == 4 && !parse_u64(argv[3], &pow))) {
        snprintf(out, n, "ERR TONE usage: TONE <f_hz> [asf] [pow]");
        return;
    }
    if (dds.fsysclk_hz == 0) { snprintf(out, n, "ERR TONE NOT_INIT"); return; }
    if (asf > 0x3FFFu || pow > 0xFFFFu) { snprintf(out, n, "ERR TONE PARAM"); return; }

    ChirpTrain_Stop();   /* a tone replaces any chirp, so stop its triggers */
#if DDS_DRY_RUN
    /* Same limit the driver applies: stay below 0.4 f_SYSCLK */
    if (f > (dds.fsysclk_hz * 2u) / 5u) { snprintf(out, n, "ERR TONE PARAM"); return; }
#else
    ad9910_status_t st = ad9910_set_single_tone(&dds, f, (uint16_t)asf, (uint16_t)pow);
    if (st != AD9910_OK) { snprintf(out, n, "ERR TONE %s", status_str(st)); return; }
#endif

    /* Report the FTW and the frequency it really produces (quantised to
     * f_SYSCLK / 2^32, about 0.23 Hz at 1 GHz). */
    uint32_t ftw = ad9910_freq_to_ftw(&dds, f);
    char fa[32];
    ftw_to_hz_str(ftw, fa, sizeof(fa));
    dds_mode = MODE_TONE;
    snprintf(out, n, "OK TONE FTW 0x%08lX FACT %s ASF %lu%s",
             (unsigned long)ftw, fa, (unsigned long)asf, dry_tag());
}

/* CHIRP <f_start_hz> <f_stop_hz> <t_ns> <SAW|TRI> */
static void cmd_chirp(int argc, char **argv, char *out, size_t n)
{
    uint64_t f0, f1, t_ns;
    ad9910_chirp_cfg_t cfg;

    if (argc != 5 || !parse_u64(argv[1], &f0) || !parse_u64(argv[2], &f1) ||
        !parse_u64(argv[3], &t_ns) || t_ns == 0) {
        snprintf(out, n, "ERR CHIRP usage: CHIRP <f0_hz> <f1_hz> <t_ns> <SAW|TRI>");
        return;
    }
    if (strcmp(argv[4], "SAW") == 0)      cfg.mode = AD9910_CHIRP_TRIGGERED_SAW;
    else if (strcmp(argv[4], "TRI") == 0) cfg.mode = AD9910_CHIRP_TRIANGLE_FREE;
    else { snprintf(out, n, "ERR CHIRP mode must be SAW or TRI"); return; }

    cfg.f_start_hz = f0;
    cfg.f_stop_hz  = f1;
    cfg.t_chirp_s  = (double)t_ns * 1e-9;   /* integer ns over the link, seconds in the driver */

    ad9910_chirp_result_t res;
    ChirpTrain_Stop();   /* never retune the ramp while triggers are running */
#if DDS_DRY_RUN
    ad9910_status_t st = ad9910_plan_chirp(&dds, &cfg, &res);      /* maths only */
#else
    ad9910_status_t st = ad9910_configure_chirp(&dds, &cfg, &res); /* maths + SPI + IO_UPDATE */
#endif
    if (st != AD9910_OK) {
        snprintf(out, n, "ERR CHIRP %s", status_str(st));
        return;
    }

    dds_mode = (cfg.mode == AD9910_CHIRP_TRIGGERED_SAW) ? MODE_SAW : MODE_TRI;
    saw_sweep_ns = (dds_mode == MODE_SAW) ? (uint64_t)llround(res.t_chirp_actual_s * 1e9) : 0u;

    /* Report what the rounding actually produced, as integers (see the
     * newlib-nano note at the top). T in ns, slope in Hz/s. */
    char t_str[21], s_str[21];
    u64_to_str((uint64_t)llround(res.t_chirp_actual_s * 1e9), t_str);
    u64_to_str((uint64_t)llround(res.slope_hz_per_s), s_str);
    snprintf(out, n, "OK CHIRP P %u STEP %lu NSTEPS %lu T_NS %s SLOPE %s%s",
             (unsigned)res.rate_p, (unsigned long)res.step,
             (unsigned long)res.n_steps, t_str, s_str, dry_tag());
}

/* ============================================================================
 * Chirp triggering: all DRCTL edges come from TIM1 (chirp_train.c)
 * ========================================================================== */

/* Shared by TRAIN and BURST. n = 0 means continuous. */
static void start_train(const char *name, uint32_t count, uint64_t period_ns,
                        char *out, size_t n)
{
    if (dds_mode != MODE_SAW) {
        snprintf(out, n, "ERR %s no SAW chirp loaded", name);
        return;
    }
    if (train_init != CT_OK) {
        snprintf(out, n, "ERR %s %s", name, ct_str(train_init));
        return;
    }

    ct_plan_t plan;
    ct_status_t st = ChirpTrain_Plan(period_ns, count, &plan);
    if (st != CT_OK) {
        snprintf(out, n, "ERR %s %s", name, ct_str(st));
        return;
    }

    /* Compare the ACHIEVED period (after rounding to timer ticks) with the
     * loaded sweep. An edge during a sweep is ignored by the AD9910, so a
     * too-short period would silently drop chirps. */
    if (plan.period_ps < (saw_sweep_ns + DDS_TRAIN_MARGIN_NS) * 1000u) {
        char t[21];
        snprintf(out, n, "ERR %s period must exceed sweep %s ns + %lu ns",
                 name, u64_to_str(saw_sweep_ns, t), (unsigned long)DDS_TRAIN_MARGIN_NS);
        return;
    }

    st = ChirpTrain_Start(&plan);
    if (st != CT_OK) {
        snprintf(out, n, "ERR %s %s", name, ct_str(st));
        return;
    }

    /* Report the achieved period and the register values, so the GUI (and
     * the report) can show exactly what the timer is doing. */
    char per[32];
    ps_to_ns_str(plan.period_ps, per, sizeof(per));
    if (count == 0u) {
        snprintf(out, n, "OK TRAIN PERIOD_NS %s PSC %lu ARR %lu CCR1 %lu%s",
                 per, (unsigned long)plan.psc, (unsigned long)plan.arr,
                 (unsigned long)plan.ccr, dry_tag());
    } else {
        snprintf(out, n, "OK BURST N %lu PERIOD_NS %s PSC %lu ARR %lu CCR1 %lu RCR %lu%s",
                 (unsigned long)count, per, (unsigned long)plan.psc,
                 (unsigned long)plan.arr, (unsigned long)plan.ccr,
                 (unsigned long)plan.rcr, dry_tag());
    }
}

/* TRAIN <period_ns> : continuous chirps until STOP */
static void cmd_train(int argc, char **argv, char *out, size_t n)
{
    uint64_t period;
    if (argc != 2 || !parse_u64(argv[1], &period)) {
        snprintf(out, n, "ERR TRAIN usage: TRAIN <period_ns>");
        return;
    }
    start_train("TRAIN", 0u, period, out, n);
}

/* BURST <n> <period_ns> : exactly n chirps, then the timer stops itself */
static void cmd_burst(int argc, char **argv, char *out, size_t n)
{
    uint64_t count, period;
    if (argc != 3 || !parse_u64(argv[1], &count) || !parse_u64(argv[2], &period) ||
        count == 0u) {
        snprintf(out, n, "ERR BURST usage: BURST <n >= 1> <period_ns>");
        return;
    }
    if (count > ChirpTrain_MaxBurst()) {
        snprintf(out, n, "ERR BURST n max %lu", (unsigned long)ChirpTrain_MaxBurst());
        return;
    }
    start_train("BURST", (uint32_t)count, period, out, n);
}

/* STOP : stop any train or burst; DRCTL returns low */
static void cmd_stop(char *out, size_t n)
{
    ChirpTrain_Stop();
    snprintf(out, n, "OK STOP%s", dry_tag());
}

/* TRIG : one chirp, implemented as a burst of 1. The period only needs to
 * hold the ~1 us pulse; the timer stops itself after the single edge. */
static void cmd_trig(char *out, size_t n)
{
    if (ChirpTrain_IsRunning()) {
        snprintf(out, n, "ERR TRIG train running, send STOP first");
        return;
    }
    if (dds_mode != MODE_SAW) {
        snprintf(out, n, "ERR TRIG no SAW chirp loaded");
        return;
    }
    if (train_init != CT_OK) {
        snprintf(out, n, "ERR TRIG %s", ct_str(train_init));
        return;
    }
    ct_plan_t plan;
    ct_status_t st = ChirpTrain_Plan(2u * DDS_TRIG_PULSE_NS, 1u, &plan);
    if (st == CT_OK) st = ChirpTrain_Start(&plan);
    if (st != CT_OK) {
        snprintf(out, n, "ERR TRIG %s", ct_str(st));
        return;
    }
    snprintf(out, n, "OK TRIG%s", dry_tag());
}

/* HOLD <0|1> */
static void cmd_hold(int argc, char **argv, char *out, size_t n)
{
    uint64_t h;
    if (argc != 2 || !parse_u64(argv[1], &h) || h > 1u) {
        snprintf(out, n, "ERR HOLD usage: HOLD <0|1>");
        return;
    }
    if (dds_mode != MODE_SAW && dds_mode != MODE_TRI) {
        snprintf(out, n, "ERR HOLD no chirp loaded");
        return;
    }
#if !DDS_DRY_RUN
    if (dds.drhold.port == NULL) { snprintf(out, n, "ERR HOLD pin not wired"); return; }
    ad9910_drg_hold(&dds, h != 0u);
#endif
    snprintf(out, n, "OK HOLD %lu%s", (unsigned long)h, dry_tag());
}

/* WREG <addr> <value> : raw write for debugging */
static void cmd_wreg(int argc, char **argv, char *out, size_t n)
{
    uint64_t addr, val;
    if (argc != 3 || !parse_u64(argv[1], &addr) || !parse_u64(argv[2], &val)) {
        snprintf(out, n, "ERR WREG usage: WREG <addr> <value>");
        return;
    }
    uint8_t len = (addr <= 0x1Fu) ? ad9910_reg_len((uint8_t)addr) : 0u;
    if (len == 0) { snprintf(out, n, "ERR WREG bad address"); return; }
    if (len < 8 && (val >> (8u * len)) != 0u) {
        snprintf(out, n, "ERR WREG value wider than register");
        return;
    }
#if DDS_DRY_RUN
    snprintf(out, n, "ERR WREG not available in DRYRUN");
#else
    ad9910_status_t st = ad9910_write_reg(&dds, (uint8_t)addr, val);
    if (st != AD9910_OK) { snprintf(out, n, "ERR WREG %s", status_str(st)); return; }
    ad9910_io_update(&dds);

    /* Keep the driver's shadow copies true. Without this, the next TONE or
     * CHIRP would rebuild CFR1..3 from stale shadows and undo this write. */
    if (addr == AD9910_REG_CFR1) dds.cfr1_shadow = (uint32_t)val;
    if (addr == AD9910_REG_CFR2) dds.cfr2_shadow = (uint32_t)val;
    if (addr == AD9910_REG_CFR3) dds.cfr3_shadow = (uint32_t)val;
    snprintf(out, n, "OK WREG");
#endif
}

/* RREG <addr> : raw read for debugging. Needs 4-wire mode, which DDS INIT
 * sets as its first configuration step. */
static void cmd_rreg(int argc, char **argv, char *out, size_t n)
{
    uint64_t addr;
    if (argc != 2 || !parse_u64(argv[1], &addr)) {
        snprintf(out, n, "ERR RREG usage: RREG <addr>");
        return;
    }
    uint8_t len = (addr <= 0x1Fu) ? ad9910_reg_len((uint8_t)addr) : 0u;
    if (len == 0) { snprintf(out, n, "ERR RREG bad address"); return; }
#if DDS_DRY_RUN
    snprintf(out, n, "ERR RREG not available in DRYRUN");
#else
    uint64_t v;
    ad9910_status_t st = ad9910_read_reg(&dds, (uint8_t)addr, &v);
    if (st != AD9910_OK) { snprintf(out, n, "ERR RREG %s", status_str(st)); return; }

    /* Print as hex in the register's own width, split into 32-bit halves
     * because %lX only covers 32 bits in newlib-nano. */
    unsigned long hi = (unsigned long)(v >> 32), lo = (unsigned long)(v & 0xFFFFFFFFu);
    if (len == 8)      snprintf(out, n, "OK RREG 0x%02X 0x%08lX%08lX", (unsigned)addr, hi, lo);
    else if (len == 4) snprintf(out, n, "OK RREG 0x%02X 0x%08lX", (unsigned)addr, lo);
    else               snprintf(out, n, "OK RREG 0x%02X 0x%04lX", (unsigned)addr, lo);
#endif
}

/* ============================================================================
 * PROBE : hardware bring-up test that does not depend on the SPI peripheral
 * ============================================================================
 * Takes SCK/SDIO/SDO away from SPI2, drives them by hand, and runs two read
 * frames straight after a MASTER_RESET:
 *
 *   A. 3-wire read of CFR3. After reset the AD9910 answers on SDIO (the
 *      STM32's MOSI line), so we release MOSI and listen on it. Expected
 *      0x1F3F4000. This needs only power, CS, SCLK and SDIO to work.
 *   B. Write CFR1 = 0x00000002 (4-wire mode), pulse IO_UPDATE, then read
 *      CFR1 back while listening on BOTH SDIO and SDO. IO_UPDATE only acts
 *      if the AD9910 core is clocked, so this tests the clock as well.
 *      Expected: SDO = 0x00000002.
 *
 * Both listening pins get a pull-UP during the reads, so a line that
 * nothing drives reads 0xFFFFFFFF (not 0, which a real register can hold).
 * Mode-0 timing as for the AD9910: we change SDIO while SCLK is low, the
 * chip samples on the rising edge and shifts read data out on the falling
 * edge, so we read the pins while SCLK is high, when the data is stable.
 * ========================================================================== */
#if !DDS_DRY_RUN
static void bb_delay(void)
{
    /* ~2 us at 80 MHz: a ~100 kHz bit clock, slow enough for any wiring */
    for (volatile uint32_t i = 0; i < 40u; i++) { __NOP(); }
}

static void bb_pin_mode(GPIO_TypeDef *port, uint16_t pin, uint32_t mode, uint32_t pull)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = pin;
    g.Mode  = mode;
    g.Pull  = pull;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
}

static void bb_sck(GPIO_PinState s) { HAL_GPIO_WritePin(DDS_SCK_PORT, DDS_SCK_PIN, s); }

/* Clock out 8 bits on SDIO, MSB first */
static void bb_write_byte(uint8_t b)
{
    for (int i = 7; i >= 0; i--) {
        HAL_GPIO_WritePin(DDS_MOSI_PORT, DDS_MOSI_PIN, ((b >> i) & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        bb_delay();
        bb_sck(GPIO_PIN_SET);     /* AD9910 samples SDIO here */
        bb_delay();
        bb_sck(GPIO_PIN_RESET);   /* in a read, the chip drives the next bit here */
    }
}

/* Clock in 32 bits, sampling SDIO and SDO together */
static void bb_read32(uint32_t *sdio, uint32_t *sdo)
{
    uint32_t a = 0, b = 0;
    for (int i = 0; i < 32; i++) {
        bb_delay();
        bb_sck(GPIO_PIN_SET);
        a = (a << 1) | (HAL_GPIO_ReadPin(DDS_MOSI_PORT, DDS_MOSI_PIN) == GPIO_PIN_SET);
        b = (b << 1) | (HAL_GPIO_ReadPin(DDS_MISO_PORT, DDS_MISO_PIN) == GPIO_PIN_SET);
        bb_delay();
        bb_sck(GPIO_PIN_RESET);
    }
    *sdio = a;
    *sdo  = b;
}

/* One full read frame: instruction byte out, 32 bits in */
static void bb_read_frame(uint8_t addr, uint32_t *sdio, uint32_t *sdo)
{
    bb_pin_mode(DDS_MOSI_PORT, DDS_MOSI_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(DDS_CS_PORT, DDS_CS_PIN, GPIO_PIN_RESET);
    bb_write_byte((uint8_t)(0x80u | (addr & 0x1Fu)));
    bb_pin_mode(DDS_MOSI_PORT, DDS_MOSI_PIN, GPIO_MODE_INPUT, GPIO_PULLUP);   /* let the chip drive SDIO */
    bb_read32(sdio, sdo);
    HAL_GPIO_WritePin(DDS_CS_PORT, DDS_CS_PIN, GPIO_PIN_SET);
}

static void cmd_probe(char *out, size_t n)
{
    uint32_t a_sdio, a_sdo, b_sdio, b_sdo;

    ChirpTrain_Stop();

    /* Hand the pins over from SPI2 to software */
    __HAL_SPI_DISABLE(&DDS_SPI_HANDLE);
    HAL_GPIO_WritePin(DDS_SCK_PORT, DDS_SCK_PIN, GPIO_PIN_RESET);
    bb_pin_mode(DDS_SCK_PORT, DDS_SCK_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    bb_pin_mode(DDS_MISO_PORT, DDS_MISO_PIN, GPIO_MODE_INPUT, GPIO_PULLUP);
    HAL_GPIO_WritePin(DDS_CS_PORT, DDS_CS_PIN, GPIO_PIN_SET);

    /* Known state: 3-wire mode, all registers at default */
    ad9910_master_reset(&dds);
    dds.fsysclk_hz = 0;          /* the chip is no longer configured */
    dds_mode = MODE_NONE;
    saw_sweep_ns = 0;

    /* A: 3-wire read of CFR3 */
    bb_read_frame(AD9910_REG_CFR3, &a_sdio, &a_sdo);

    /* B: CFR1 = 0x00000002 (SDIO input only), IO_UPDATE, read CFR1 back */
    bb_pin_mode(DDS_MOSI_PORT, DDS_MOSI_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    HAL_GPIO_WritePin(DDS_CS_PORT, DDS_CS_PIN, GPIO_PIN_RESET);
    bb_write_byte(AD9910_REG_CFR1);
    bb_write_byte(0x00); bb_write_byte(0x00); bb_write_byte(0x00); bb_write_byte(0x02);
    HAL_GPIO_WritePin(DDS_CS_PORT, DDS_CS_PIN, GPIO_PIN_SET);
    ad9910_io_update(&dds);
    bb_read_frame(AD9910_REG_CFR1, &b_sdio, &b_sdo);

    int lock = HAL_GPIO_ReadPin(DDS_PLLLOCK_PORT, DDS_PLLLOCK_PIN) == GPIO_PIN_SET;
    int drov = HAL_GPIO_ReadPin(DDS_DROVER_PORT, DDS_DROVER_PIN) == GPIO_PIN_SET;

    /* Give the pins back to SPI2, exactly as CubeMX set them up */
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = DDS_SPI_AF;
    g.Pin = DDS_SCK_PIN | DDS_MOSI_PIN;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DDS_SCK_PORT, &g);
    g.Pin = DDS_MISO_PIN;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(DDS_MISO_PORT, &g);

    snprintf(out, n, "OK PROBE A_SDIO 0x%08lX A_SDO 0x%08lX B_SDIO 0x%08lX B_SDO 0x%08lX LOCK %d DROVER %d",
             (unsigned long)a_sdio, (unsigned long)a_sdo,
             (unsigned long)b_sdio, (unsigned long)b_sdo, lock, drov);
}
#endif

/* ============================================================================
 * Dispatcher
 * ========================================================================== */
int DdsCmd_Handle(char *line, char *out, size_t out_size)
{
    /* Split into words in place: each space becomes '\0' and argv[i]
     * points at the start of each word. No extra memory is used. */
    char *argv[MAX_ARGS];
    int   argc = 0;
    char *p = line;
    while (*p != '\0' && argc < MAX_ARGS) {
        while (*p == ' ') *p++ = '\0';      /* skip (and cut at) spaces */
        if (*p == '\0') break;
        argv[argc++] = p;
        while (*p != '\0' && *p != ' ') p++;
    }
    if (argc == 0) return 0;

    const char *c = argv[0];

    if (strcmp(c, "DDS") == 0) {
        if (argc == 2 && strcmp(argv[1], "INIT") == 0)   { cmd_init(out, out_size);   return 1; }
        if (argc == 2 && strcmp(argv[1], "STATUS") == 0) { cmd_status(out, out_size); return 1; }
        if (argc == 2 && strcmp(argv[1], "RESET") == 0)  { cmd_reset(out, out_size);  return 1; }
        snprintf(out, out_size, "ERR DDS usage: DDS <INIT|STATUS|RESET>");
        return 1;
    }
    if (strcmp(c, "TONE")  == 0) { cmd_tone(argc, argv, out, out_size);  return 1; }
    if (strcmp(c, "CHIRP") == 0) { cmd_chirp(argc, argv, out, out_size); return 1; }
    if (strcmp(c, "TRIG")  == 0) { cmd_trig(out, out_size);              return 1; }
    if (strcmp(c, "TRAIN") == 0) { cmd_train(argc, argv, out, out_size); return 1; }
    if (strcmp(c, "BURST") == 0) { cmd_burst(argc, argv, out, out_size); return 1; }
    if (strcmp(c, "STOP")  == 0) { cmd_stop(out, out_size);              return 1; }
    if (strcmp(c, "HOLD")  == 0) { cmd_hold(argc, argv, out, out_size);  return 1; }
    if (strcmp(c, "WREG")  == 0) { cmd_wreg(argc, argv, out, out_size);  return 1; }
    if (strcmp(c, "RREG")  == 0) { cmd_rreg(argc, argv, out, out_size);  return 1; }
    if (strcmp(c, "PROBE") == 0) {
#if DDS_DRY_RUN
        snprintf(out, out_size, "ERR PROBE not available in DRYRUN");
#else
        cmd_probe(out, out_size);
#endif
        return 1;
    }

    return 0;   /* not a DDS command */
}
