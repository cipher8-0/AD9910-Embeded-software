# DDS Command Layer Explainer

This layer connects the USB link you already tested to the AD9910 driver. The PC sends a text line, the firmware turns it into driver calls, and the reply reports exactly what was programmed.

---

## 1. Where it sits

```
PC (GUI / test_dds.py)
   |  "CHIRP 100000000 200000000 100000 SAW\n"
   v
USB interrupt -> CDC_Receive_FS -> ring buffer            (usb_cmd.c, unchanged)
   |
main loop: UsbCmd_Poll() -> handle_line()                 (usb_cmd.c)
   |  PING / LED / ECHO / STAT handled here as before
   |  anything else ->
   v
DdsCmd_Handle()                                           (dds_cmd.c, new)
   |  split words, parse numbers, check ranges, check state
   v
ad9910_configure_chirp() -> SPI writes -> IO_UPDATE       (ad9910.c)
   |
   v
reply "OK CHIRP P 1 STEP 17180 NSTEPS 25000 T_NS 100000 SLOPE 1000007614493"
```

Everything from `handle_line()` down runs in the **main loop**, not in the USB interrupt. That is what makes it safe for driver calls to block (SPI transfers, `HAL_Delay()` during init): the USB interrupt keeps storing incoming bytes in the ring buffer while the main loop is busy.

---

## 2. Commands

| Command | What happens | Example reply |
|---|---|---|
| `DDS INIT` | Reset, 4-wire SPI, PLL to 1 GHz, readback checks | `OK INIT FSYS 1000000000` |
| `DDS STATUS` | Reports state, no hardware writes | `OK STATUS INIT 1 FSYS 1000000000 MODE SAW LOCK 1` |
| `DDS RESET` | MASTER_RESET; everything refused until next INIT | `OK RESET` |
| `TONE <f_hz> [asf] [pow]` | Single tone via profile 0 | `OK TONE FTW 0x1999999A FACT 100000000.093 ASF 16383` |
| `CHIRP <f0> <f1> <t_ns> <SAW\|TRI>` | Plans and loads a DRG ramp | `OK CHIRP P 1 STEP 17180 NSTEPS 25000 T_NS 100000 SLOPE 1000007614493` |
| `TRIG` | DRCTL rising edge (SAW only) | `OK TRIG` |
| `HOLD <0\|1>` | DRHOLD pin | `OK HOLD 1` |
| `WREG <addr> <value>` | Raw write + IO_UPDATE (debug) | `OK WREG` |
| `RREG <addr>` | Raw read (debug) | `OK RREG 0x02 0x1F3F4000` |

Errors always look like `ERR <CMD> <reason>`, where the reason is either a usage hint or a driver status (`PARAM`, `SPI`, `PLL`, `COMMS`, `NOT_INIT`).

**FACT** in the TONE reply is the frequency actually produced. The FTW is an integer, so the output is quantised to f_SYSCLK / 2^32 ≈ 0.23 Hz; asking for 100 MHz gives 100 000 000.093 Hz.

**The CHIRP reply** gives the achieved chirp after integer rounding (section 9 of the driver explainer), not the requested one. The GUI should display these values.

Chirp time is sent as **integer nanoseconds** rather than a decimal like `100e-6`. Integers are unambiguous to parse, and the GUI can convert from whatever units it shows.

---

## 3. State checking

`dds_cmd.c` remembers what the AD9910 is currently set up for (`NONE`, `TONE`, `SAW`, `TRI`). This lets it refuse commands that make no sense:

- TONE and CHIRP before `DDS INIT` → `ERR ... NOT_INIT` (f_SYSCLK is unknown, so no FTW can be computed)
- TRIG unless a sawtooth chirp is loaded
- HOLD unless some chirp is loaded

`DDS RESET` clears the state, because after MASTER_RESET the PLL is off and nothing is configured.

**WREG keeps the shadows in sync.** If you write CFR1, CFR2 or CFR3 by hand, the driver's RAM copy is updated too. Otherwise the next TONE or CHIRP would rebuild the register from the old copy and silently undo your write.

---

## 4. Dry-run mode

The pin mapping isn't known yet and the DDS board isn't connected, so `dds_board.h` starts with `DDS_DRY_RUN 1`. In this mode:

- parsing, range checks, state checks and all the maths (FTW, chirp planner) run for real;
- nothing touches SPI or the AD9910 pins;
- `DDS INIT` just sets f_SYSCLK = 25 MHz × 40 so the maths has a value to work with;
- every reply ends in `DRYRUN`, so it's obvious nothing reached hardware;
- WREG and RREG are refused, since they only make sense with a real chip.

This lets you test the whole path now: GUI → USB → parser → planner → reply. When the hardware is ready, only `dds_board.h` changes.

The driver also builds without any SPI peripheral enabled in CubeMX. The HAL only defines the SPI types when `HAL_SPI_MODULE_ENABLED` is set in `stm32l4xx_hal_conf.h`, so `ad9910.h` declares a placeholder type when it isn't, and the SPI calls in `ad9910.c` go through two small wrappers that return an error.

---

## 5. Printing numbers with newlib-nano

CubeIDE links **newlib-nano**, a reduced C library. Its `printf`/`snprintf` leave out, by default:

- floating point (`%f`), which prints nothing unless enabled with a linker flag;
- 64-bit integers (`%llu`, `%llX`).

So the command layer never uses either. Doubles are rounded to integers first (`llround`), 64-bit decimals go through `u64_to_str()`, 64-bit hex is printed as two 32-bit halves, and TONE's fractional Hz is computed with integer maths (`ftw_to_hz_str()`). 32-bit values use `%lu`/`%lX` with an `(unsigned long)` cast, which nano handles.

---

## 6. Integration steps

1. Copy `ad9910.c/.h`, `dds_cmd.c/.h` and `dds_board.h` into the project (`Core/Src` and `Core/Inc`). Replace the older `usb_cmd.c/.h` with the updated ones.
2. In `main.c`:

   ```c
   /* USER CODE BEGIN Includes */
   #include "dds_cmd.h"

   /* USER CODE BEGIN 2 (after LedTest_Init) */
   DdsCmd_Init();   /* fills in the driver config; does not touch the AD9910 */
   ```

3. Build, flash, and run `python test_dds.py COM5`. In dry-run mode all 16 checks should pass.

`DDS INIT` is deliberately **not** run automatically at startup. The GUI (or you) sends it once the DDS board is powered, and gets a clear error if something is wrong, instead of the firmware stalling silently at boot.

---

## 7. Going live later

1. In CubeMX, enable the SPI peripheral connected to the AD9910 (settings in section 2 of the driver explainer), and set the control pins as GPIO outputs (CS initial level High) and inputs.
2. Fill in the pin macros in `dds_board.h`, and set `DDS_SPI_HANDLE` to that peripheral's handle (e.g. `hspi2`).
3. Set `DDS_ICP_CODE` to the charge pump code your loop filter was designed for.
4. Set `DDS_DRY_RUN` to 0 and rebuild.
5. Run `test_dds.py` again, then check with `RREG 2` that CFR3 now holds the PLL settings.

---

## 8. Changes made to the driver in this step

- **`ad9910_plan_chirp()`** split out of `ad9910_configure_chirp()`. It does the maths only and never touches hardware. `configure_chirp()` now calls it, then writes the registers. Dry-run mode uses the planner alone.
- **`n_steps`** added to `ad9910_chirp_result_t`, so the reply can report it.
- **SPI wrappers** `spi_tx()` / `spi_rx()` let the driver compile before SPI is enabled in CubeMX (section 4).

The register logic is unchanged.
