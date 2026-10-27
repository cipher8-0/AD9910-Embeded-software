/* ============================================================================
 * usb_cmd.c : receive text commands over USB CDC, act on them, reply
 * ============================================================================
 *
 * DATA FLOW, PC -> BOARD
 * ----------------------
 *  1. Python writes "LED 5\n" to the COM port.
 *  2. The PC's USB driver packs those bytes into a USB packet and sends it
 *     to the board's OUT endpoint (OUT = direction host -> device).
 *  3. The STM32 USB peripheral receives the packet and raises an interrupt.
 *  4. ST's USB stack handles the interrupt and calls CDC_Receive_FS() in
 *     usbd_cdc_if.c, which calls our UsbCmd_OnRx().
 *  5. UsbCmd_OnRx() copies the bytes into a ring buffer and returns.
 *  6. Later, the main loop calls UsbCmd_Poll(), which takes bytes out of the
 *     ring buffer, builds a line, and runs the command.
 *
 * DATA FLOW, BOARD -> PC
 * ----------------------
 *  1. The command handler calls reply("OK LED 5").
 *  2. usb_send() hands the bytes to CDC_Transmit_FS().
 *  3. The USB stack queues them on the IN endpoint (IN = device -> host).
 *  4. The PC polls the device, collects the packet, and Python's readline()
 *     returns "OK LED 5".
 *
 * IMPORTANT: USB DELIVERS PACKETS, NOT LINES
 * ------------------------------------------
 * A full-speed CDC packet carries at most 64 bytes. A long command may
 * arrive split over several packets, or one packet may contain two commands.
 * That is why we collect bytes one at a time and only act when we see '\n'.
 * ========================================================================== */

#include "usb_cmd.h"
#include "led_test.h"      /* LedTest_Set(), LedTest_Enable() */
#include "dds_cmd.h"       /* DdsCmd_Handle(): AD9910 commands */
#include "usbd_cdc_if.h"   /* CDC_Transmit_FS() and USB stack types */
#include <string.h>        /* memcpy, strcmp, strncmp */
#include <stdio.h>         /* snprintf */
#include <stdlib.h>        /* strtol */

/* hUsbDeviceFS is the USB device "handle": one big struct holding the state
 * of the whole USB stack. CubeMX creates it in usb_device.c. "extern" means:
 * this variable is defined in another file, we just want to use it here. */
extern USBD_HandleTypeDef hUsbDeviceFS;

#define RX_RING_SIZE   2048u  /* bytes of storage between ISR and main loop */
#define LINE_MAX       256u   /* longest command line we accept (incl. '\0') */
#define TX_TIMEOUT_MS  50u    /* how long to wait for the previous send */

/* ----------------------------------------------------------------------------
 * RING BUFFER (circular FIFO) between the USB interrupt and the main loop
 * ----------------------------------------------------------------------------
 * rx_head = index where the ISR writes the NEXT byte.
 * rx_tail = index where the main loop reads the NEXT byte.
 * head == tail        -> buffer empty.
 * head + 1 == tail    -> buffer full (we sacrifice one slot so that "full"
 *                        and "empty" look different).
 * Indices wrap to 0 at the end (the "% RX_RING_SIZE"), hence "ring".
 *
 * Only the ISR writes rx_head and only the main loop writes rx_tail, so the
 * two never fight over the same variable. On a Cortex-M4, reading or writing
 * an aligned 32-bit variable is a single instruction, so no locking is needed.
 *
 * "volatile" tells the compiler: this can change behind your back (the ISR
 * changes it). Without it, the compiler might keep rx_head in a register
 * inside the main loop and never see new data arrive.
 * -------------------------------------------------------------------------- */
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint32_t rx_head, rx_tail;
static volatile uint32_t rx_overflow;   /* count of bytes dropped when full */

/* The line currently being assembled by UsbCmd_Poll() */
static char     line[LINE_MAX];
static uint32_t line_len;        /* number of characters stored so far */
static uint8_t  line_too_long;   /* set if the line overflowed LINE_MAX */

/* Transmit buffer. CDC_Transmit_FS() does NOT copy your data; it just starts
 * the transfer and returns, and the hardware reads from this memory later.
 * So the buffer must still exist after our function returns. A local
 * variable would disappear, so it has to be static (lives forever). */
static uint8_t  tx_buf[LINE_MAX + 16];

/* ============================================================================
 * UsbCmd_OnRx : store incoming bytes. Runs INSIDE the USB interrupt.
 * ========================================================================== */
void UsbCmd_OnRx(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        /* Where head would move to after writing this byte */
        uint32_t next = (rx_head + 1u) % RX_RING_SIZE;

        if (next == rx_tail) {
            /* Buffer full: the main loop has fallen behind. We cannot wait
             * inside an interrupt, so drop the remaining bytes and count them.
             * The PC can read this count with the STAT command. */
            rx_overflow++;
            return;
        }
        rx_ring[rx_head] = buf[i];   /* store the byte */
        rx_head = next;              /* then publish it by advancing head */
    }
}

/* ============================================================================
 * rx_pop : take one byte out of the ring buffer (main loop side).
 * Returns 1 and writes the byte to *c if one was available, 0 if empty.
 * ========================================================================== */
static int rx_pop(uint8_t *c)
{
    if (rx_tail == rx_head) return 0;           /* empty */
    *c = rx_ring[rx_tail];                      /* read the byte */
    rx_tail = (rx_tail + 1u) % RX_RING_SIZE;    /* then free the slot */
    return 1;
}

/* ============================================================================
 * usb_send : send raw bytes to the PC.
 * ========================================================================== */
static void usb_send(const char *s, uint32_t len)
{
    /* pClassData points to the CDC class's own state struct. It is NULL
     * until the PC has finished enumerating the device (recognised it and
     * chosen a configuration). If it is NULL, there is nobody to send to. */
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL || len == 0 || len > sizeof(tx_buf)) return;

    /* TxState is 1 while a previous transmission is still in progress and
     * goes back to 0 when the PC has collected it. We only have one tx_buf,
     * so we must not overwrite it until the old data has gone out. */
    uint32_t t0 = HAL_GetTick();                     /* ms since boot */
    while (hcdc->TxState != 0) {
        if (HAL_GetTick() - t0 > TX_TIMEOUT_MS) {
            /* The PC is not reading (e.g. no program has the COM port
             * open). Give up on this reply rather than freezing forever. */
            return;
        }
    }

    memcpy(tx_buf, s, len);                  /* copy into the persistent buffer */
    CDC_Transmit_FS(tx_buf, (uint16_t)len);  /* start the USB IN transfer */
}

/* ============================================================================
 * reply : send a text string followed by '\n' (so Python's readline()
 * knows where the reply ends).
 * ========================================================================== */
static void reply(const char *s)
{
    char out[LINE_MAX + 16];

    /* snprintf formats into 'out' and never writes past sizeof(out).
     * It returns how many characters it WANTED to write. */
    int n = snprintf(out, sizeof(out), "%s\n", s);
    if (n < 0) return;                                  /* formatting error */
    if ((uint32_t)n >= sizeof(out)) n = sizeof(out) - 1; /* was truncated */

    usb_send(out, (uint32_t)n);
}

/* ============================================================================
 * handle_line : decide what a complete command line means and act on it.
 * 's' is a normal C string with the '\n' already removed.
 * ========================================================================== */
static void handle_line(char *s)
{
    char r[32];   /* small scratch buffer for building replies */

    /* strcmp returns 0 when the two strings are identical */
    if (strcmp(s, "PING") == 0) {
        reply("PONG");
    }
    /* strncmp(s, "LED ", 4) checks only the first 4 characters */
    else if (strncmp(s, "LED ", 4) == 0) {
        /* strtol converts text to a number. 'end' is set to the first
         * character it could NOT convert, which lets us detect junk:
         *   "LED 5"   -> m = 5, end points at '\0'   (good)
         *   "LED x"   -> end == s+4, nothing converted (bad)
         *   "LED 5x"  -> end points at 'x'           (bad) */
        char *end;
        long m = strtol(s + 4, &end, 10);   /* base 10 */
        if (end == s + 4 || *end != '\0' || m < 0 || m > 7) {
            reply("ERR range 0-7");
            return;
        }
        LedTest_Enable(1);           /* Q4 on, so LED cathodes are grounded */
        LedTest_Set((uint8_t)m);     /* 3-bit mask to the three LED pins */
        snprintf(r, sizeof(r), "OK LED %ld", m);
        reply(r);
    }
    else if (strncmp(s, "ECHO ", 5) == 0) {
        reply(s + 5);   /* s + 5 skips past "ECHO " to the payload text */
    }
    else if (strcmp(s, "STAT") == 0) {
        snprintf(r, sizeof(r), "RXOVF %lu", (unsigned long)rx_overflow);
        reply(r);
    }
    else {
        /* Not one of the basic commands above: offer it to the DDS command
         * layer. It returns 1 if it recognised the command (and wrote a
         * reply into 'out'), 0 if the command is unknown to it as well. */
        char out[LINE_MAX];
        if (DdsCmd_Handle(s, out, sizeof(out))) {
            reply(out);
        } else {
            reply("ERR unknown");
        }
    }
}

/* ============================================================================
 * UsbCmd_Poll : called continuously from main(). Drains the ring buffer,
 * builds lines, and dispatches each complete line to handle_line().
 * ========================================================================== */
void UsbCmd_Poll(void)
{
    uint8_t c;

    /* Keep going while there are bytes waiting */
    while (rx_pop(&c)) {

        /* Windows tools often send "\r\n". Ignore '\r' so both work. */
        if (c == '\r') continue;

        if (c == '\n') {
            /* End of a command line */
            if (line_too_long) {
                reply("ERR too long");
            } else if (line_len > 0) {       /* ignore empty lines */
                line[line_len] = '\0';       /* terminate to make a C string */
                handle_line(line);
            }
            /* Reset for the next line */
            line_len = 0;
            line_too_long = 0;
        }
        else if (line_len < LINE_MAX - 1) {
            /* Normal character: append it (leave 1 slot for '\0') */
            line[line_len++] = (char)c;
        }
        else {
            /* Line is longer than we can store. Keep discarding characters
             * until '\n', then report the error once. */
            line_too_long = 1;
        }
    }
}
