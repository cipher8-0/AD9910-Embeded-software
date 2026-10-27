#!/usr/bin/env python3
"""Exercise the DDS command layer over USB (works in DRYRUN and live).

Usage:   python test_dds.py COM5
Requires: pip install pyserial

Each test sends one command and checks whether the reply starts with the
expected word (OK or ERR). Error cases are included on purpose: they prove
the firmware rejects bad input instead of programming garbage.
"""
import sys
import serial

# (command, expected reply prefix, what it checks)
TESTS = [
    ("DDS RESET",                               "OK",  "start from a known state"),
    ("TONE 100000000",                          "ERR", "refused before init"),
    ("DDS INIT",                                "OK",  "PLL / clock configuration"),
    ("DDS STATUS",                              "OK",  "reports init state and f_SYSCLK"),
    ("TONE 100000000",                          "OK",  "100 MHz full scale"),
    ("TONE 50000000 8192 0",                    "OK",  "50 MHz half amplitude"),
    ("TONE 500000000",                          "ERR", "above 0.4 f_SYSCLK"),
    ("TONE 100000000 20000",                    "ERR", "ASF above 14 bits"),
    ("TRIG",                                    "ERR", "no sawtooth loaded yet"),
    ("CHIRP 100000000 200000000 100000 SAW",    "OK",  "100 to 200 MHz in 100 us"),
    ("TRIG",                                    "OK",  "start one sweep (TIM1 burst of 1)"),
    ("TRAIN 200000",                            "OK",  "continuous, 200 us: PSC 0 ARR 15999 CCR1 15920"),
    ("STOP",                                    "OK",  "stop the train"),
    ("TRAIN 100500",                            "ERR", "period shorter than 100 us sweep + 1 us"),
    ("TRAIN 2000000",                           "OK",  "2 ms needs PSC 2 (ARR 53332)"),
    ("BURST 5 200000",                          "OK",  "exactly 5 chirps, RCR 4"),
    ("BURST 0 200000",                          "ERR", "a burst needs at least 1 chirp"),
    ("STOP",                                    "OK",  "stop (harmless if already stopped)"),
    ("CHIRP 100000000 200000000 10000000 TRI",  "OK",  "10 ms triangle"),
    ("TRAIN 200000",                            "ERR", "trains only make sense for SAW"),
    ("CHIRP 200000000 100000000 100000 SAW",    "ERR", "start above stop"),
    ("CHIRP 100000000 200000000 1 SAW",         "ERR", "1 ns is impossibly fast"),
    ("CHIRP 1 2 3 SIDEWAYS",                    "ERR", "bad mode word"),
    ("DDS STATUS",                              "OK",  "mode should now be TRI"),
]


def cmd(ser, text):
    ser.write((text + "\n").encode())
    return ser.readline().decode(errors="replace").rstrip("\r\n")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    ser = serial.Serial(sys.argv[1], timeout=2)   # DDS INIT can take a few ms
    ser.reset_input_buffer()

    fails = 0
    for text, expect, why in TESTS:
        reply = cmd(ser, text)
        ok = reply.startswith(expect)
        fails += not ok
        mark = "pass" if ok else "FAIL"
        print(f"[{mark}] {text:<42} -> {reply}")
        if not ok:
            print(f"       expected {expect}: {why}")

    ser.close()
    print(f"\n{len(TESTS) - fails}/{len(TESTS)} passed")
    sys.exit(0 if fails == 0 else 1)


if __name__ == "__main__":
    main()