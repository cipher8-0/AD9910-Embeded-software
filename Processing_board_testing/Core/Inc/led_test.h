#ifndef LED_TEST_H
#define LED_TEST_H

#include <stdint.h>

/* Processing board LED test (STM32L476VETx)
 * CTRL_LEDS -> PB3  (base of Q4, low-side switch for all three LEDs)
 * LED0      -> PC3  (D4, red)
 * LED1      -> PC14 (D5, green)
 * LED2      -> PC15 (D6, blue)
 * An LED lights only when its GPIO is HIGH *and* CTRL_LEDS is HIGH.
 */

void LedTest_Init(void);
void LedTest_Set(uint8_t mask);   /* bit0 = LED0, bit1 = LED1, bit2 = LED2 */
void LedTest_Enable(uint8_t on);  /* drives CTRL_LEDS (Q4) */
void LedTest_RunOnce(void);       /* one pass of the test sequence (~15 s) */

#endif /* LED_TEST_H */
