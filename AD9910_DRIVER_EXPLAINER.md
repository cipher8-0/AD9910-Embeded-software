# AD9910 Driver Explainer

This document explains what `ad9910.c` / `ad9910.h` do and why. The code comments tell you *what a line does*; this file gives the background needed to understand *why it is done that way*. Datasheet references are to AD9910 Rev. E.

---

## 1. The big picture

The STM32 never generates the RF signal itself. It only **configures** the AD9910, which then produces the waveform on its own at 1 GSPS. The driver is therefore mostly about moving numbers into the AD9910's registers at the right moment.

There are two kinds of link between the two chips:

| Link | Signals | Used for |
|---|---|---|
| SPI | SCLK, SDIO (STM32 MOSI), SDO (STM32 MISO), CS | Reading and writing registers: frequency, PLL, ramp settings |
| GPIO | IO_UPDATE, MASTER_RESET, DRCTL, DRHOLD, IO_RESET (outputs); DROVER, PLL_LOCK (inputs) | Real-time control and status, where timing matters more than data |

The driver is layered. Each layer only uses the one below it:

```
ad9910_init / set_single_tone / configure_chirp     <- what the GUI firmware will call
ad9910_freq_to_ftw                                   <- Hz to register units
ad9910_io_update / ad9910_master_reset               <- control pulses
ad9910_write_reg / ad9910_read_reg                   <- one register per SPI frame
pin_write / pin_read / cs_low / cs_high              <- STM32 HAL GPIO
```

---

## 2. STM32 setup the driver expects (CubeMX)

**SPI peripheral** (for example SPI1):

| Setting | Value | Why |
|---|---|---|
| Mode | Full-Duplex Master | We need MISO for readback (4-wire mode) |
| Hardware NSS | Disable | CS is a normal GPIO the driver controls |
| Data size | 8 bits | The AD9910 protocol is byte oriented |
| First bit | MSB first | AD9910 default (CFR1 LSB_FIRST = 0) |
| CPOL | Low | AD9910 samples SDIO on the **rising** edge of SCLK and drives SDO on the **falling** edge. CPOL = 0, CPHA = 1 Edge (SPI mode 0) matches this. |
| CPHA | 1 Edge | |
| Prescaler | Start around 1 to 5 MHz | The AD9910 accepts up to 70 MHz, but jumper wires between boards do not. Raise it later once readback is reliable. |

**GPIO**: CS, IO_UPDATE, MASTER_RESET, DRCTL (and DRHOLD, IO_RESET if wired) as push-pull outputs. Set CS's initial level to High in CubeMX so the AD9910 never sees a false transaction at power-up. DROVER and PLL_LOCK (if wired) as inputs.

The driver includes `main.h`, which CubeMX generates and which pulls in the right HAL for the chip, so the same files build on the Nucleo F446RE and the L476 board.

---

## 3. One SPI frame = one register access

Every access is a single frame with CS held low from start to end:

```
CS    ‾‾\__________________________________________/‾‾
        [instruction][data MSB] ... [data LSB]
         1 byte       2, 4 or 8 bytes (depends on register)
```

The **instruction byte**:

```
bit:   7     6   5   4  3  2  1  0
      R/W    x   x   A4 A3 A2 A1 A0
```

R/W = 0 for write, 1 for read. A4..A0 is the register address. `addr & 0x1F` keeps a bad address from leaking into bit 7.

The AD9910 knows how many data bytes to expect from the address, so the STM32 must send exactly that many. That is what `ad9910_reg_len()` is for.

**Byte order.** Registers are sent most significant byte first. For a 4-byte value the loop computes shifts of 24, 16, 8, 0, taking the top byte first. Reading does the reverse: each received byte shifts the previous ones up by 8.

**How the SPI peripheral does it.** `HAL_SPI_Transmit()` writes each byte into the SPI data register (`SPIx->DR`); the hardware shifts it out bit by bit on MOSI while generating SCLK, then raises the TXE flag so the next byte can go in. SPI is always full duplex, so to *receive*, the master must still clock: `HAL_SPI_Receive()` sends dummy bytes and keeps what arrives on MISO. These HAL calls are **blocking** (polled): the CPU waits in a loop until the frame is done. At a few MHz a frame takes microseconds, so this is fine.

**3-wire vs 4-wire.** After reset the AD9910 is in 3-wire mode, where SDIO is bidirectional. The STM32's SPI expects separate MOSI and MISO, so `ad9910_init()` first sets CFR1 `SDIO_INPUT_ONLY`, which moves read data onto the SDO pin. This requires SDO to be wired to MISO on the inter-board connection.

---

## 4. Buffered registers and IO_UPDATE

This is the most important AD9910 concept for the driver.

An SPI write does **not** change the chip's behaviour. It lands in an **I/O buffer**. The buffered values are copied into the active registers only when the IO_UPDATE pin is pulsed high.

Why this matters: a chirp needs four registers (limits, step, rate, CFR2). If each took effect as soon as it was written, the output would pass through meaningless intermediate states. With buffering, all four are written, then one IO_UPDATE applies them in the same clock cycle.

**Pulse width.** The AD9910 samples IO_UPDATE using its internal SYNC_CLK (f_SYSCLK / 4), and the pulse must stay high for more than one SYNC_CLK period. Right after reset the PLL is off and the input /2 divider is active, so with a 25 MHz crystal f_SYSCLK is only 12.5 MHz and SYNC_CLK's period is 320 ns. `short_delay()` gives about 1 µs, scaled from `SystemCoreClock` so it stays valid whatever the STM32 clock is.

---

## 5. Shadow registers

The driver keeps RAM copies of CFR1, CFR2 and CFR3 (`cfr1_shadow` etc.). When a function needs to change one bit, it edits the shadow and writes the whole register, instead of reading the register over SPI, modifying it, and writing it back. This is faster and does not depend on readback working.

The shadows must always match the chip, which is why `ad9910_master_reset()` resets them to the default values: after a hardware reset the chip is at defaults, so the copies must be too.

---

## 6. Initialisation sequence

`ad9910_init()` runs these steps in order. Each one depends on the one before.

1. **Idle levels.** CS high, all pulse lines low, so nothing is triggered by accident.
2. **MASTER_RESET.** Known register state regardless of what ran before.
3. **4-wire mode.** CFR1 = `SDIO_INPUT_ONLY`, then IO_UPDATE.
4. **Comms smoke test.** Read CFR3. It has not been written since reset, so it must equal the datasheet default. If this fails, the problem is SPI wiring, SPI mode, or the SDO line, and nothing later can work. Only the *defined* bits are compared (`AD9910_CFR3_DEFINED_MASK`): the datasheet shows the unused ("open") bits as 1 in the default value, and the chip's actual readback of open bits is not specified.
5. **Clock configuration (CFR3)** for the selected GA4 arm:

   | Arm | Input divider | PLL | VCO SEL | N | f_SYSCLK |
   |---|---|---|---|---|---|
   | A: crystal | bypassed | on | VCO5 | 40 | 25 MHz × 40 = 1 GHz |
   | B: external 25 MHz | bypassed | on | VCO5 | 40 | 1 GHz |
   | C: direct ~1 GHz | bypassed | off | 111 (bypass) | unused | = REFCLK |

   Arms A and B use identical register settings; they differ only in which clock source the hardware presents. The divider is bypassed in A/B because the loop filter was designed for a 25 MHz phase detector frequency.

   The driver checks N is in 12..127 and f_SYSCLK is within the PLL's 420 MHz to 1 GHz range (datasheet limits) before writing anything.
6. **PLL lock.** The PLL_LOCK pin is **latched** (datasheet, "PLL Lock Indication"), and may already read 1 from the bypassed state after reset. The driver therefore pulses the PFD reset bit first, which clears the latch, then enables the PLL normally and waits for a genuine lock with a 10 ms timeout.
7. **Base CFR2.** Defaults with the PDCLK output turned off. The parallel port is unused, and a toggling clock pin is a spur source you don't want in GA4 measurements.
8. **Verify.** Read CFR2 back and compare, proving writes land at the final clock rate.

Only after all of that succeeds is `fsysclk_hz` set. Every higher-level function checks it and returns `AD9910_ERR_NOT_INIT` if it is still 0.

---

## 7. Frequency tuning word

The DDS phase accumulator is 32 bits. Each SYSCLK cycle it adds the FTW; the output frequency is how fast it wraps:

```
f_out = FTW × f_SYSCLK / 2^32        so        FTW = round(2^32 × f_out / f_SYSCLK)
```

At 1 GHz, one FTW step is 1e9 / 2^32 ≈ 0.233 Hz.

`ad9910_freq_to_ftw()` does this in 64-bit integers. The Cortex-M4 FPU is single precision (24-bit mantissa), which cannot represent a 32-bit FTW exactly. `f << 32` fits in 64 bits for any f below 4.29 GHz, and adding half the divisor before dividing rounds to nearest.

Frequencies are limited to 0.4 × f_SYSCLK (400 MHz at 1 GHz). Above that, the DAC image near f_SYSCLK − f_out gets too close to the wanted signal to filter out.

---

## 8. Single tone

The AD9910 has 8 **profiles** (registers 0x0E to 0x15), each holding a frequency, phase and amplitude. The PROFILE[2:0] pins select which one is active. The driver uses profile 0, so those pins must be 000 on the board.

Profile layout (64 bits):

```
[63:62] open   [61:48] ASF (14-bit amplitude)   [47:32] POW (phase)   [31:0] FTW
```

`ad9910_set_single_tone()` turns the DRG off (it would override the profile frequency), enables amplitude-from-profile, packs the fields, writes CFR2 and profile 0, then pulses IO_UPDATE once.

Note the casts in the packing: `(uint64_t)asf << 48`. Without the cast, the 16-bit value is promoted to a 32-bit `int`, and shifting that by 48 is undefined behaviour in C, which in practice loses all the bits.

---

## 9. Chirps with the Digital Ramp Generator (DRG)

The DRG is a second accumulator that adds a step to the frequency at a fixed interval, producing a staircase that approximates a linear ramp. Three registers define it:

| Register | Upper 32 bits | Lower 32 bits |
|---|---|---|
| DR_LIMIT (0x0B) | upper limit (FTW of f_stop) | lower limit (FTW of f_start) |
| DR_STEP (0x0C) | decrement step | increment step |
| DR_RATE (0x0D, 32-bit) | [31:16] N, down interval | [15:0] P, up interval |

The relations (datasheet, digital ramp generator section):

```
step interval   dt = 4P / f_SYSCLK          (DRG clock = SYNC_CLK = f_SYSCLK / 4)
freq per step   df = STEP × f_SYSCLK / 2^32
chirp time      T  = (span / STEP) × dt,    span = FTW_hi − FTW_lo
```

### The planner

Two integers (P and STEP) must be chosen to hit one target (T). There are many valid pairs, and the planner balances two goals:

- **Small P** gives fine time steps, so the staircase is closer to a smooth ramp.
- **Large STEP** makes rounding STEP to an integer matter less. The slope error is about 0.5 / STEP.

So the planner finds the smallest P that still allows STEP ≥ `AD9910_DRG_MIN_STEP` (1000, about 0.05% slope error), rounding P **up** because rounding down would push STEP below the minimum. It then computes STEP for that P, rounded to nearest, and reports the chirp time and slope actually achieved.

### Worked example (f_SYSCLK = 1 GHz)

**100 → 200 MHz in 100 µs**

- FTW_lo = 429 496 730, FTW_hi = 858 993 459, span = 429 496 729
- P = ceil(1000 × 1e9 × 100e-6 / (4 × 429 496 729)) = ceil(0.058) = **1**, so dt = 4 ns
- STEP = 429 496 729 × 4 × 1 / (1e9 × 100e-6) = 17 179.87 → **17 180**, so df ≈ 4 kHz per step
- 25 000 steps × 4 ns = **100.000 µs**, slope ≈ 1.000008e12 Hz/s (error ≈ 8 ppm)

**Same span in 10 ms**

- P = **6** (dt = 24 ns), STEP = **1031**, 416 583 steps
- actual T = 9.998 ms, slope error ≈ 0.02%

Here P had to rise above 1 to keep STEP ≥ 1000, trading time resolution for slope accuracy.

### Chirp modes (CFR2 no-dwell bits)

- **Triggered sawtooth** (no-dwell high only). A rising edge on DRCTL starts an up-ramp. On reaching the upper limit the output **snaps** to the lower limit and waits for the next rising edge. In this mode DRCTL is edge-sensitive; its level doesn't matter. That is why `ad9910_chirp_trigger()` drives it low, then high, so there is always a fresh edge.
- **Free triangle** (both no-dwell bits). The ramp runs up and down between the limits continuously, and DRCTL is ignored.

When the DRG is enabled with DRCTL = 0, the output initialises at the lower limit, which is why `configure_chirp()` sets DRCTL low before its IO_UPDATE. The chirp then sits at f_start until triggered.

**Amplitude.** In chirp mode the profile amplitude is disabled, so the output is full scale. Single tone uses the ASF you pass in. Keep this in mind when comparing power levels between the two modes.

**DROVER** is high whenever the ramp output is at either limit. In sawtooth mode it is high while waiting and low during the sweep, so it marks the chirp window exactly. That makes it a good scope trigger, or an EXTI interrupt input if firmware needs to know when a sweep ends.

---

## 10. Using the driver in main.c

```c
#include "ad9910.h"

ad9910_t dds = {
    .hspi         = &hspi1,
    .cs           = { GPIOx, GPIO_PIN_x },   /* fill from Justin's schematic */
    .io_update    = { GPIOx, GPIO_PIN_x },
    .master_reset = { GPIOx, GPIO_PIN_x },
    .drctl        = { GPIOx, GPIO_PIN_x },
    .drover       = { GPIOx, GPIO_PIN_x },
    .pll_lock     = { NULL, 0 },            /* not wired: blind 5 ms wait */
    .arm          = AD9910_ARM_A_XTAL_PLL,
    .refclk_hz    = 25000000u,
    .pll_n        = 40,
    .icp_code     = 0,                      /* set to your loop filter design */
};

if (ad9910_init(&dds) != AD9910_OK) { /* report the status code */ }
ad9910_set_single_tone(&dds, 100000000ULL, 0x3FFF, 0);   /* 100 MHz, full scale */
```

Any optional pin left as `{ NULL, 0 }` is simply skipped by the GPIO helpers.

---

## 11. Review: what changed from the version you had, and why

| Change | Reason |
|---|---|
| `#include "main.h"` instead of `stm32f4xx_hal.h` | Builds unchanged on both the F446 and the L476 |
| PFD reset pulse before waiting for PLL lock | PLL_LOCK is latched and can read 1 before the PLL has really locked. Without this, init could report success with an unlocked PLL. |
| Readback comparisons masked to defined bits | The datasheet default for CFR3 shows open bits as 1; real readback of open bits is unspecified, so an unmasked compare could fail on a working board |
| N (12 to 127) and f_SYSCLK (420 MHz to 1 GHz) checked | Datasheet PLL limits; stops invalid settings reaching the chip |
| `fsysclk_hz` only set on success, plus `AD9910_ERR_NOT_INIT` | Calling tone/chirp before init previously divided by zero |
| STEP range-checked as a double before the cast | A value above 2^32 cast to `uint32_t` wraps silently and could pass the old `step > span` check |
| Optional pins: `port = NULL` instead of `has_pll_lock_pin` | Lines such as DRHOLD may not be wired; writing to a NULL port would crash |
| `short_delay()` scaled from `SystemCoreClock` | The old fixed loop assumed 180 MHz (F446); the L476 runs at 80 MHz |
| `DR_RATE` moved into the 4-byte group in `reg_len` | Same result, simpler switch |
| DROVER comment corrected | It is high while *at* a limit (including while waiting), not only a pulse on reaching one |

The core logic (frame format, byte order, FTW maths, chirp planner, register packing, no-dwell modes) was already correct.

---

## 12. Things to confirm on the hardware

- **Pin mapping** from the processing board to the inter-board connector, and whether SDO is wired to MISO (needed for 4-wire readback).
- **PROFILE[2:0]** are tied or driven low, since single tone uses profile 0.
- **ICP code** matching your loop filter design (R1 = 120 Ω, C1 = 33 nF, C2 = 6.8 nF). The Icp values are in datasheet Table 9 (000 = 212 µA up to 111 = 387 µA).
- **VCO5 margin at 1 GHz.** The VCO band edges vary part to part (datasheet Figure 33). If lock is unreliable, check which band this chip actually needs.
- **SYNC_CLK output.** CFR2 enables it by default, which toggles a pin at 250 MHz. The driver leaves it on for now; clearing `AD9910_CFR2_SYNC_CLK_EN` is worth trying if it shows up as a spur in the GA4 measurements.
