#include "led_test.h"
#include "stm32l4xx_hal.h"

#define CTRL_PORT  GPIOB
#define CTRL_PIN   GPIO_PIN_3

#define LED_PORT   GPIOC
#define LED0_PIN   GPIO_PIN_13
#define LED1_PIN   GPIO_PIN_14
#define LED2_PIN   GPIO_PIN_15
#define LED_ALL    (LED0_PIN | LED1_PIN | LED2_PIN)

/* PC14/PC15 are the LSE oscillator pins. LSEON lives in the backup domain,
 * which survives a normal reset. If earlier firmware (e.g. the micromouse
 * code) enabled the LSE, the pins stay owned by the oscillator and GPIO
 * writes do nothing. Clear it explicitly. */
static void release_lse_pins(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    if (RCC->BDCR & (RCC_BDCR_LSEON | RCC_BDCR_LSEBYP)) {
        RCC->BDCR &= ~RCC_BDCR_LSEON;
        while (RCC->BDCR & RCC_BDCR_LSERDY) { }
        RCC->BDCR &= ~RCC_BDCR_LSEBYP;   /* only writable once LSEON = 0 */
    }
}

void LedTest_Init(void)
{
    GPIO_InitTypeDef g = {0};

    release_lse_pins();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Preload output latches low so nothing flashes on mode change */
    HAL_GPIO_WritePin(CTRL_PORT, CTRL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_ALL, GPIO_PIN_RESET);

    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;   /* PC14/PC15 must stay <= 2 MHz */

    g.Pin = CTRL_PIN;                /* overrides the JTDO/SWO default on PB3 */
    HAL_GPIO_Init(CTRL_PORT, &g);

    g.Pin = LED_ALL;
    HAL_GPIO_Init(LED_PORT, &g);
}

void LedTest_Set(uint8_t mask)
{
    uint32_t set = 0, clr = 0;

    if (mask & 0x1) set |= LED0_PIN; else clr |= LED0_PIN;
    if (mask & 0x2) set |= LED1_PIN; else clr |= LED1_PIN;
    if (mask & 0x4) set |= LED2_PIN; else clr |= LED2_PIN;

    LED_PORT->BSRR = set | (clr << 16);  /* all three change in one write */
}

void LedTest_Enable(uint8_t on)
{
    HAL_GPIO_WritePin(CTRL_PORT, CTRL_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LedTest_RunOnce(void)
{
    /* 1. Gate check: anodes HIGH, Q4 OFF. Board should stay DARK.
     *    If LEDs light here, Q4 is shorted C-E or the emitter path is wrong. */
    LedTest_Enable(0);
    LedTest_Set(0x7);
    HAL_Delay(1500);

    /* 2. Q4 ON: all three should light. */
    LedTest_Enable(1);
    HAL_Delay(1500);
    LedTest_Set(0x0);
    HAL_Delay(400);

    /* 3. One at a time, slow: confirms pin -> colour mapping (R, G, B). */
    for (int i = 0; i < 3; i++) {
        LedTest_Set((uint8_t)(1u << i));
        HAL_Delay(800);
    }

    /* 4. Fast chase, 4 laps. */
    for (int lap = 0; lap < 4; lap++) {
        for (int i = 0; i < 3; i++) {
            LedTest_Set((uint8_t)(1u << i));
            HAL_Delay(120);
        }
    }

    /* 5. 3-bit binary counter 0..7. */
    for (uint8_t n = 0; n < 8; n++) {
        LedTest_Set(n);
        HAL_Delay(500);
    }

    /* 6. Master blink via CTRL_LEDS only (anodes held HIGH). */
    LedTest_Set(0x7);
    for (int i = 0; i < 10; i++) {
        LedTest_Enable((uint8_t)(i & 1) ? 0 : 1);
        HAL_Delay(200);
    }

    LedTest_Set(0x0);
    LedTest_Enable(0);
    HAL_Delay(1000);
}
