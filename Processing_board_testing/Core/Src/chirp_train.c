/* ============================================================================
 * chirp_train.c : hardware-timed DRCTL triggers using TIM1 CH1 (PE9)
 * ============================================================================
 *
 * TIM1 IN ONE PARAGRAPH
 * ---------------------
 * TIM1 is a hardware counter clocked from the APB2 timer clock (80 MHz).
 * The prescaler PSC divides that clock; the counter CNT then counts
 * 0, 1, ... ARR and wraps to 0 (an "update event"). Channel 1 compares CNT
 * with CCR1 every tick and drives pin PE9 from the result, with no CPU
 * involvement. In PWM mode 2:
 *        CNT <  CCR1  ->  PE9 low
 *        CNT >= CCR1  ->  PE9 high
 * so each period has one rising edge (at CCR1) and one falling edge (at the
 * wrap). We keep the high time at about 1 us and place it at the end of the
 * period.
 *
 * REGISTERS USED (TIM1 base address 0x40012C00 on the L476)
 *   CR1    control: CEN (counter enable), OPM (one-pulse), ARPE (ARR preload)
 *   PSC    prescaler           ARR    auto-reload        CCR1   compare value
 *   RCR    repetition counter  CNT    the counter itself
 *   EGR    event generation: UG forces an update event by software
 *   CCMR1  channel 1 mode (PWM mode 2, CCR1 preload)
 *   CCER   channel enable CC1E and polarity CC1P
 *   BDTR   MOE: master output enable, needed by advanced timers only
 *
 * PRELOAD
 * -------
 * PSC, ARR, CCR1 and RCR are double-buffered: a write goes into a "preload"
 * register, and the hardware copies it into the active ("shadow") register
 * only at the next update event. This prevents a half-updated period. To
 * make new values active immediately, we force an update event with EGR.UG.
 * ========================================================================== */

#include "chirp_train.h"
#include "dds_board.h"
#include <stddef.h>

#define TICKS_MAX_PER_REG   65536ull                  /* PSC+1 and ARR+1 are 16-bit */
#define TICKS_MAX_PERIOD    (TICKS_MAX_PER_REG * TICKS_MAX_PER_REG)
#define PERIOD_NS_MAX       100000000000ull           /* 100 s: keeps the maths in 64 bits */

/* Largest RCR value the hardware implements. Measured in ChirpTrain_Init()
 * on real hardware; assumed 16-bit for dry-run. */
static uint32_t rcr_max = 0xFFFFu;

#if DDS_TRAIN_HW

#include "main.h"

#ifndef HAL_TIM_MODULE_ENABLED
#error "TIM1 is not enabled in CubeMX: enable TIM1 CH1 PWM on PE9, or set DDS_TRAIN_HW to 0 in dds_board.h"
#endif

/* Created by CubeMX in main.c (MX_TIM1_Init). Holds TIM1's register base
 * address in .Instance plus HAL bookkeeping state. */
extern TIM_HandleTypeDef DDS_TIM_HANDLE;
#define TIMX   (DDS_TIM_HANDLE.Instance)   /* TIM_TypeDef *, i.e. the registers */

static ct_status_t init_status = CT_ERR_CLOCK;   /* until ChirpTrain_Init() succeeds */

/* Actual clock feeding TIM1. Timers on an APB bus run at PCLK if that bus's
 * prescaler is /1, and at 2 x PCLK otherwise (reference manual, clock tree).
 * PPRE2 bits 13:11 of RCC->CFGR hold the APB2 prescaler; its top bit set
 * means "divided". */
static uint32_t tim1_clock_hz(void)
{
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    return (RCC->CFGR & RCC_CFGR_PPRE2_2) ? 2u * pclk2 : pclk2;
}

ct_status_t ChirpTrain_Init(void)
{
    ChirpTrain_Stop();

    /* Refuse to run if the tick maths would be wrong */
    if (tim1_clock_hz() != DDS_TIM_CLK_HZ) {
        init_status = CT_ERR_CLOCK;
        return init_status;
    }

    /* Enforce the configuration this module depends on, so a mistake in the
     * CubeMX settings cannot silently break the timing. */
    TIMX->CR1 |= TIM_CR1_ARPE;                               /* ARR preload on */
    TIMX->CCMR1 = (TIMX->CCMR1 & ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M))
                | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0  /* 0111 = PWM mode 2 */
                | TIM_CCMR1_OC1PE;                           /* CCR1 preload on */
    TIMX->CCER &= ~TIM_CCER_CC1P;                            /* active high */

    /* Find the repetition counter width: write all ones, read back. Bits
     * that are not implemented in hardware read as 0. */
    TIMX->RCR = 0xFFFFu;
    rcr_max = TIMX->RCR;
    TIMX->RCR = 0;

    init_status = CT_OK;
    return init_status;
}

ct_status_t ChirpTrain_Start(const ct_plan_t *p)
{
    if (init_status != CT_OK) return init_status;

    /* HAL_TIM_PWM_Start fails if the HAL thinks the channel is still busy
     * (a finished burst stops the counter but not the HAL state), so always
     * stop first. */
    ChirpTrain_Stop();

    /* 1. Write the new values. They go into the preload registers. */
    TIMX->PSC  = p->psc;
    TIMX->ARR  = p->arr;
    TIMX->CCR1 = p->ccr;
    TIMX->RCR  = p->rcr;

    /* 2. One-pulse mode for bursts: hardware clears CEN (stops the counter)
     *    at the update event, which with RCR = N-1 happens after N periods. */
    if (p->n != 0u) TIMX->CR1 |=  TIM_CR1_OPM;
    else            TIMX->CR1 &= ~TIM_CR1_OPM;

    /* 3. Force an update event: copies PSC/ARR/CCR1/RCR into the active
     *    registers now and resets CNT and the repetition counter. UG also sets
     *    the update flag UIF; we use no interrupts, so just clear it. */
    TIMX->EGR = TIM_EGR_UG;
    TIMX->SR  = ~TIM_SR_UIF;

    /* 4. Start one tick before the compare value so the first rising edge
     *    (first chirp) happens immediately instead of almost a period later.
     *    The first period is shorter, but it still contains exactly one edge,
     *    so a burst still produces exactly N chirps. */
    TIMX->CNT = p->ccr - 1u;

    /* 5. HAL_TIM_PWM_Start sets CC1E (connect channel to pin), MOE (TIM1's
     *    master output enable, which keeps all outputs off until set), and
     *    finally CEN (start counting). From here the hardware runs alone. */
    if (HAL_TIM_PWM_Start(&DDS_TIM_HANDLE, DDS_TIM_CHANNEL) != HAL_OK) {
        return CT_ERR_HAL;
    }
    return CT_OK;
}

void ChirpTrain_Stop(void)
{
    /* Clears CC1E and MOE and stops the counter. PE9 is then no longer driven
     * by the timer and the pull-down (set in CubeMX) holds DRCTL low. */
    HAL_TIM_PWM_Stop(&DDS_TIM_HANDLE, DDS_TIM_CHANNEL);
    TIMX->CR1 &= ~TIM_CR1_CEN;
}

int ChirpTrain_IsRunning(void)
{
    return (TIMX->CR1 & TIM_CR1_CEN) != 0u;
}

#else  /* ------------------- DDS_TRAIN_HW = 0: planning only ------------------ */

ct_status_t ChirpTrain_Init(void)                 { return CT_OK; }
ct_status_t ChirpTrain_Start(const ct_plan_t *p)  { (void)p; return CT_OK; }
void        ChirpTrain_Stop(void)                 { }
int         ChirpTrain_IsRunning(void)            { return 0; }

#endif /* DDS_TRAIN_HW */

uint32_t ChirpTrain_MaxBurst(void)
{
    return rcr_max + 1u;
}

/* ============================================================================
 * Planner: period_ns, n -> PSC, ARR, CCR1, RCR (worked examples in the chat
 * explanation: 200 us -> PSC 0, ARR 15999, CCR1 15920)
 * ========================================================================== */
ct_status_t ChirpTrain_Plan(uint64_t period_ns, uint32_t n, ct_plan_t *p)
{
    const uint64_t clk = DDS_TIM_CLK_HZ;

    if (period_ns == 0u || period_ns > PERIOD_NS_MAX) return CT_ERR_PERIOD;
    if (n != 0u && (n - 1u) > rcr_max) return CT_ERR_COUNT;

    /* Timer clocks per period, rounded to nearest (80 per us at 80 MHz) */
    uint64_t ticks = (period_ns * clk + 500000000ull) / 1000000000ull;
    if (ticks < 2u || ticks > TICKS_MAX_PERIOD) return CT_ERR_PERIOD;

    /* PSC+1: smallest divider that lets ARR+1 fit in 16 bits.
     * Smallest = finest resolution, (PSC+1) x 12.5 ns. */
    uint64_t div = (ticks + TICKS_MAX_PER_REG - 1u) / TICKS_MAX_PER_REG;   /* ceil */

    /* ARR+1: counter ticks per period, rounded to nearest */
    uint64_t top = (ticks + div / 2u) / div;
    if (top > TICKS_MAX_PER_REG) top = TICKS_MAX_PER_REG;
    if (top < 2u) return CT_ERR_PERIOD;

    /* High-pulse length in counter ticks (~DDS_TRIG_PULSE_NS). Must leave at
     * least one low tick so there is an edge each period. */
    uint64_t pulse_clk = ((uint64_t)DDS_TRIG_PULSE_NS * clk) / 1000000000ull;
    uint64_t pulse = (pulse_clk + div / 2u) / div;
    if (pulse < 1u) pulse = 1u;
    if (pulse >= top) pulse = top / 2u;

    p->psc = (uint32_t)(div - 1u);
    p->arr = (uint32_t)(top - 1u);
    p->ccr = (uint32_t)(top - pulse);           /* rising edge here */
    p->n   = n;
    p->rcr = (n == 0u) ? 0u : (n - 1u);

    /* Achieved period: div x top timer clocks. In picoseconds so a 12.5 ns
     * tick is exact: 1e12 / 80e6 = 12 500 ps per clock. */
    p->period_ps = div * top * 1000000ull / (clk / 1000000ull);
    return CT_OK;
}
