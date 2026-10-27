#!/usr/bin/env python3
"""USB CDC test for the processing board.

Usage:   python test_usb.py COM5          (Windows)
         python test_usb.py /dev/ttyACM0  (Linux)
List ports: python -m serial.tools.list_ports
Requires: pip install pyserial

What it does:
  Stage 1  PING          -> confirms two-way communication works at all
  Stage 3  LED 0..7      -> visual check, each command should change the LEDs
  Stage 2  500 x ECHO    -> random text sent and compared byte for byte,
                            reports errors and a rough speed figure
  Extra    BOGUS / STAT  -> checks error handling and dropped-byte counter
"""
import random   # to generate random test text
import string   # gives us ready-made letter/digit alphabets
import sys      # command-line arguments and exit codes
import time     # delays and timing

import serial   # pyserial: opens and talks to COM ports


def cmd(ser, text):
    """Send one command line and return the board's one-line reply.

    Adds the '\n' the board waits for, converts str -> bytes for sending,
    then reads until '\n' comes back (or the timeout expires, in which
    case readline() returns whatever it has, possibly empty).
    """
    ser.write((text + "\n").encode())              # str -> bytes, then send
    raw = ser.readline()                           # bytes up to and incl. '\n'
    return raw.decode(errors="replace").rstrip("\r\n")  # bytes -> str, strip newline


def main():
    # sys.argv[0] is the script name, sys.argv[1] should be the port name
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    # Open the COM port. With CDC there is no real UART behind it, so the
    # baud rate setting is ignored; USB always runs at 12 Mbit/s.
    # timeout=1 means readline() gives up after 1 s instead of hanging.
    ser = serial.Serial(sys.argv[1], timeout=1)

    time.sleep(0.2)          # let the port settle after opening
    ser.reset_input_buffer() # throw away any stale bytes from earlier runs

    # ---------------- Stage 1: link check ----------------
    r = cmd(ser, "PING")
    print(f"PING -> {r!r}")  # !r shows quotes, so an empty reply is visible
    if r != "PONG":
        print("FAIL: no PONG. Check port, firmware, and that main loop calls UsbCmd_Poll().")
        sys.exit(1)

    # ---------------- Stage 3: LED control (visual) ----------------
    # Count 0..7 in binary on the three LEDs. Watch the board.
    for m in range(8):
        print(f"LED {m} -> {cmd(ser, f'LED {m}')}")
        time.sleep(0.3)      # slow enough to see each pattern
    cmd(ser, "LED 0")        # all off at the end

    # ---------------- Stage 2: echo integrity + speed ----------------
    n_lines, errors, nbytes = 500, 0, 0
    alphabet = string.ascii_letters + string.digits   # a-z A-Z 0-9
    t0 = time.perf_counter()                          # high-resolution timer

    for i in range(n_lines):
        # Random length from 1 to 200 characters. Varying the length means
        # some lines fit in one 64-byte USB packet and others span several,
        # which exercises the board's line reassembly.
        payload = "".join(random.choices(alphabet, k=random.randint(1, 200)))
        r = cmd(ser, "ECHO " + payload)
        nbytes += 2 * len(payload)                    # counted both directions
        if r != payload:
            errors += 1
            if errors <= 5:                           # only print the first few
                print(f"  mismatch on line {i}: sent {len(payload)} B, got {len(r)} B")

    dt = time.perf_counter() - t0
    # This is a request/response test (send, wait for reply, repeat), so the
    # speed is limited by round-trip latency, not by USB bandwidth. Treat it
    # as a sanity figure, not the maximum the link can do.
    print(f"Echo: {n_lines - errors}/{n_lines} OK, "
          f"{nbytes / dt / 1024:.1f} kB/s round-trip payload, "
          f"{1000 * dt / n_lines:.2f} ms per command")

    # ---------------- Extra checks ----------------
    print(f"BOGUS -> {cmd(ser, 'BOGUS')!r}")  # expect 'ERR unknown'
    print(f"STAT  -> {cmd(ser, 'STAT')!r}")   # expect 'RXOVF 0'

    ser.close()
    # Exit code 0 = pass, 1 = fail (useful if you script this later)
    sys.exit(0 if errors == 0 else 1)


# Only run main() when this file is executed directly, not when imported
if __name__ == "__main__":
    main()
