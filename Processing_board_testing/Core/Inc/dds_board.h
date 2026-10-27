/* ============================================================================
 * dds_board.h : how the AD9910 is wired to THIS processing board
 * ============================================================================
 * This is the only file that changes when the wiring is known. Everything
 * else (driver, command layer) reads its hardware details from here.
 *
 * DRY-RUN MODE
 * ------------
 * With DDS_DRY_RUN = 1 the command layer parses and validates every command
 * and runs the real chirp/FTW maths, but never touches SPI or the AD9910
 * GPIOs. The replies show exactly what WOULD be programmed. This lets you
 * test the complete GUI -> USB -> firmware path now, before the pin mapping
 * is known or the DDS board is connected.
 *
 * To go live:
 *   1. In CubeMX, enable SPI2 and TIM1 CH1 (PE9), and set the AD9910
 *      control pins as GPIO outputs/inputs.
 *   2. Check the pin macros below against the wiring.
 *   3. Set DDS_DRY_RUN to 0 and rebuild.
 * ========================================================================== */
#ifndef DDS_BOARD_H
#define DDS_BOARD_H

#define DDS_DRY_RUN            0

/* ---- Clock configuration (GA4 arm A: 25 MHz crystal, PLL x40 = 1 GHz) ---- */
#define DDS_CLK_ARM            AD9910_ARM_A_XTAL_PLL
#define DDS_REFCLK_HZ          25000000u
#define DDS_PLL_N              40u
#define DDS_ICP_CODE           3u      /* 287 uA charge pump (datasheet Table 9) */

/* ---- Chirp-train timer (TIM1) -------------------------------------------
 * These are used in dry-run too, so the period maths can be tested.
 * DDS_TIM_CLK_HZ is the clock feeding TIM1's prescaler: APB2 timer clock,
 * 80 MHz with the APB2 prescaler at /1. ChirpTrain_Init() checks this
 * against the real clock tree and refuses to run if they disagree. */
#define DDS_TIM_CLK_HZ         80000000u
#define DDS_TRIG_PULSE_NS      1000u   /* DRCTL high time per trigger (~1 us) */
#define DDS_TRAIN_MARGIN_NS    1000u   /* period must exceed sweep time by this */

/* Drive TIM1 for real (1) or only plan the periods (0). Independent of
 * DDS_DRY_RUN, so DRCTL pulses can be checked on a scope at PE9 before the
 * DDS board exists: dry-run DDS + real timer. Needs TIM1 CH1 set up in
 * CubeMX; if it is not, the build stops with an explanatory #error. */
#define DDS_TRAIN_HW           1

/* CubeMX creates htim1 in main.c. chirp_train.c is written for channel 1
 * (it programs CCR1 / CCMR1 directly). */
#define DDS_TIM_HANDLE         htim1
#define DDS_TIM_CHANNEL        TIM_CHANNEL_1

#if !DDS_DRY_RUN
/* ---- SPI ------------------------------------------------------------------
 * AD9910 on SPI2: PB13 = SCK, PB14 = MISO (from AD9910 SDO),
 * PB15 = MOSI (to AD9910 SDIO), all alternate function AF5.
 * CubeMX creates the handle hspi2 in main.c; it holds the peripheral's
 * register base address (SPI2 = 0x40003800 on the L476) and settings. */
#define DDS_SPI_HANDLE         hspi2

/* The same three SPI pins, for the PROBE command, which takes them away
 * from SPI2 for a moment and drives them by hand (bit-banging). */
#define DDS_SCK_PORT           GPIOB
#define DDS_SCK_PIN            GPIO_PIN_13
#define DDS_MISO_PORT          GPIOB     /* AD9910 SDO */
#define DDS_MISO_PIN           GPIO_PIN_14
#define DDS_MOSI_PORT          GPIOB     /* AD9910 SDIO */
#define DDS_MOSI_PIN           GPIO_PIN_15
#define DDS_SPI_AF             GPIO_AF5_SPI2

/* ---- Pins the driver controls ------------------------------------------- */
#define DDS_CS_PORT            GPIOB     /* CSB, GPIO output, initial level HIGH */
#define DDS_CS_PIN             GPIO_PIN_12
#define DDS_IOUPD_PORT         GPIOC     /* IO_UPDATE */
#define DDS_IOUPD_PIN          GPIO_PIN_3
#define DDS_MRESET_PORT        GPIOB     /* MASTER_RESET */
#define DDS_MRESET_PIN         GPIO_PIN_0
/* DRCTL (PE9) belongs to TIM1 CH1, not to the driver. In alternate-function
 * mode GPIO writes have no effect on the pin, so the driver gets NULL and
 * skips it; all DRCTL edges come from chirp_train.c. */
#define DDS_DRCTL_PORT         NULL
#define DDS_DRCTL_PIN          0
#define DDS_IORESET_PORT       GPIOB     /* IO_RESET */
#define DDS_IORESET_PIN        GPIO_PIN_1
#define DDS_DRHOLD_PORT        GPIOC     /* DRHOLD */
#define DDS_DRHOLD_PIN         GPIO_PIN_5
#define DDS_DROVER_PORT        GPIOC     /* DROVER, GPIO input */
#define DDS_DROVER_PIN         GPIO_PIN_10
#define DDS_PLLLOCK_PORT       GPIOC     /* PLL_LOCK, GPIO input */
#define DDS_PLLLOCK_PIN        GPIO_PIN_0

/* ---- Pins held at a fixed level (set in CubeMX, not touched by code) ------
 * These must never float. Configure them in CubeMX as GPIO outputs with
 * initial level LOW; MX_GPIO_Init() then drives them before main() reaches
 * any DDS code:
 *   PC4 EXT_PWR_DWN  low = AD9910 powered up
 *   PC1 OSK          low (OSK is disabled in CFR1 anyway)
 *   PA0 PROFILE0, PA1 PROFILE1, PC2 PROFILE2
 *                    all low = profile 0, which single-tone mode uses
 *   PA15 RAM_SWP_OVR is an AD9910 OUTPUT: configure as GPIO input (unused). */
#endif

#endif /* DDS_BOARD_H */
