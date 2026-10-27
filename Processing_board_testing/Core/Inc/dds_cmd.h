/* ============================================================================
 * dds_cmd.h : text commands from the GUI -> AD9910 driver calls
 * ============================================================================
 * Sits between usb_cmd.c (which assembles lines from USB) and ad9910.c
 * (which talks SPI). One line in, one reply line out, same as before.
 *
 *   DDS INIT                          reset + configure the AD9910 (PLL, 4-wire SPI)
 *   DDS STATUS                        init state, f_SYSCLK, mode, PLL lock
 *   DDS RESET                         MASTER_RESET, marks the chip uninitialised
 *   TONE <f_hz> [asf] [pow]           single tone; asf 0..16383 (default 16383),
 *                                     pow 0..65535 (default 0)
 *   CHIRP <f_start_hz> <f_stop_hz> <t_ns> <SAW|TRI>
 *                                     load a linear chirp into the ramp generator
 *   TRIG                              one sawtooth chirp (TIM1 burst of 1)
 *   TRAIN <period_ns>                 continuous chirps, hardware-timed by TIM1
 *   BURST <n> <period_ns>             exactly n chirps, then TIM1 stops itself
 *   STOP                              stop TRAIN/BURST, DRCTL back low
 *   HOLD <0|1>                        freeze / resume the ramp (DRHOLD pin)
 *   WREG <addr> <value>               raw register write + IO_UPDATE (debug)
 *   RREG <addr>                       raw register read (debug)
 *   PROBE                             bit-banged bring-up test, bypasses SPI2
 *                                     (resets the chip; send DDS INIT after)
 *
 * Numbers may be decimal (100000000) or hex (0x1F). Replies start with
 * "OK <CMD>" or "ERR <CMD> <reason>", so the GUI can parse them.
 * ========================================================================== */
#ifndef DDS_CMD_H
#define DDS_CMD_H

#include <stddef.h>

/* Fill in the driver's pin/clock configuration. Call once at startup,
 * after the CubeMX MX_..._Init() functions. Does not touch the AD9910. */
void DdsCmd_Init(void);

/* Try to handle one command line. Returns 1 and writes the reply into
 * 'out' if the line was a DDS command, 0 if it is not one of ours
 * (so the caller can try other handlers or reply "ERR unknown").
 * 'line' is modified (split into words). */
int DdsCmd_Handle(char *line, char *out, size_t out_size);

#endif /* DDS_CMD_H */
