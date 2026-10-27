/* ============================================================================
 * chirp_train.h : hardware-timed DRCTL triggers using TIM1 CH1 (PE9)
 * ============================================================================
 * TIM1 runs in PWM mode 2 and produces one short high pulse per period on
 * DRCTL. Each rising edge starts one sawtooth chirp on the AD9910.
 *
 *   period = (PSC + 1) x (ARR + 1) / f_TIM          (f_TIM = 80 MHz)
 *   rising edge when the counter reaches CCR1, falling edge when it wraps
 *
 * Continuous train: timer free-runs until ChirpTrain_Stop().
 * Burst of N:       one-pulse mode + repetition counter RCR = N - 1, so the
 *                   timer stops itself after exactly N periods.
 *
 * With DDS_TRAIN_HW = 0 (dds_board.h) the planning maths runs normally but
 * the timer is never touched. With 1 (default) TIM1 runs for real, even
 * when DDS_DRY_RUN = 1, so DRCTL pulses can be checked on a scope early.
 * ========================================================================== */
#ifndef CHIRP_TRAIN_H
#define CHIRP_TRAIN_H

#include <stdint.h>

typedef enum {
    CT_OK = 0,
    CT_ERR_PERIOD,     /* period too short for the timer or above ~53 s */
    CT_ERR_COUNT,      /* burst count larger than the repetition counter allows */
    CT_ERR_CLOCK,      /* real TIM1 clock does not match DDS_TIM_CLK_HZ */
    CT_ERR_HAL         /* HAL refused to start the timer */
} ct_status_t;

/* Register values for one train, plus the period they actually produce */
typedef struct {
    uint32_t psc;        /* prescaler register  (counter ticks every PSC+1 clocks) */
    uint32_t arr;        /* auto-reload register (counter counts 0..ARR)          */
    uint32_t ccr;        /* compare register: rising edge at CNT == CCR1           */
    uint32_t rcr;        /* repetition counter: burst of RCR+1 periods             */
    uint32_t n;          /* chirps in the burst, 0 = continuous                    */
    uint64_t period_ps;  /* achieved period in picoseconds (12.5 ns = 12500 ps)    */
} ct_plan_t;

/* Configure TIM1 CH1 for this module (PWM mode 2, preload, polarity) and
 * check the timer clock. Call once after MX_TIM1_Init(). */
ct_status_t ChirpTrain_Init(void);

/* Pure maths: period and count -> register values. Never touches hardware. */
ct_status_t ChirpTrain_Plan(uint64_t period_ns, uint32_t n, ct_plan_t *plan);

/* Load a plan into TIM1 and start it (first chirp starts immediately). */
ct_status_t ChirpTrain_Start(const ct_plan_t *plan);

/* Stop the timer; DRCTL returns low (driven low, then held by the pull-down). */
void ChirpTrain_Stop(void);

/* 1 while the timer is counting. A burst clears this by itself when done. */
int ChirpTrain_IsRunning(void);

/* Largest N accepted by BURST (repetition counter range + 1). */
uint32_t ChirpTrain_MaxBurst(void);

#endif /* CHIRP_TRAIN_H */
