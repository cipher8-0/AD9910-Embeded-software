/* ============================================================================
 * usb_cmd.h : simple text command protocol over USB CDC (virtual COM port)
 * ============================================================================
 *
 * BIG PICTURE
 * -----------
 * When the board runs the CDC class, the PC sees it as a serial (COM) port.
 * The PC and board then exchange plain bytes, exactly like a UART, except the
 * bytes travel inside USB packets. We build a tiny protocol on top of that:
 * the PC sends one command per line (text ending in '\n'), the board sends
 * back one reply line.
 *
 * Commands understood by the board:
 *   PING          -> PONG            link check: "are you alive?"
 *   LED <0-7>     -> OK LED <n>      set LED mask (bit0=red, bit1=green,
 *                                    bit2=blue) and switch Q4 on
 *   ECHO <text>   -> <text>          send text back unchanged (tests data
 *                                    integrity and gives a speed figure)
 *   STAT          -> RXOVF <count>   how many received bytes were dropped
 *                                    because the receive buffer was full
 *   DDS / TONE / CHIRP / TRIG / HOLD / WREG / RREG
 *                 -> passed on to dds_cmd.c (see dds_cmd.h for the list)
 *   anything else -> ERR unknown
 *
 * TWO FUNCTIONS, TWO CONTEXTS
 * ---------------------------
 * UsbCmd_OnRx() is called from inside the USB interrupt every time a packet
 * arrives. Code in an interrupt must be short and fast, so it ONLY stores the
 * bytes.
 *
 * UsbCmd_Poll() is called over and over from the main while(1) loop. It does
 * the slow work: assembling lines, parsing commands, driving LEDs, replying.
 * ========================================================================== */

#ifndef USB_CMD_H
#define USB_CMD_H

#include <stdint.h>

/* Called from CDC_Receive_FS() in usbd_cdc_if.c (interrupt context).
 * buf = pointer to the bytes that just arrived, len = how many bytes. */
void UsbCmd_OnRx(const uint8_t *buf, uint32_t len);

/* Called from the main loop. Processes any bytes that have arrived. */
void UsbCmd_Poll(void);

#endif /* USB_CMD_H */
