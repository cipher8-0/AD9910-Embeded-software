#!/usr/bin/env python3
"""
board_gui.py : desktop GUI for the RaptorWatch processing board (STM32L476)

Talks to the board over its USB virtual COM port with the line protocol in
usb_cmd.h and dds_cmd.h. Two tabs:
  Board : LEDs and raw commands (link testing)
  DDS   : AD9910 init/status, single tone, chirp setup and triggering

Run:      python board_gui.py
Requires: pip install pyserial        (Tkinter ships with Python on Windows)

HOW THE PROGRAM IS ORGANISED
----------------------------
Unchanged from the first version: a background SerialWorker thread owns the
COM port and exchanges messages with the window through two queues, so a
slow reply (DDS INIT can take several ms) never freezes the window.

New in this version: replies are PARSED, not just logged. Every OK reply
has the form
        OK <CMD> <KEY> <VALUE> <KEY> <VALUE> ... [DRYRUN]
so parse_reply() turns it into a dictionary, and a handler per command
(on_dds_reply, on_tone_reply, on_chirp_reply) updates the readouts.

UNITS
-----
The firmware works in integer Hz, integer ns and raw register codes
(ASF 0..16383, POW 0..65535). People think in MHz, us, percent and degrees.
All conversions happen here, in the GUI, so the firmware stays simple and
the link carries exact integers.
"""

import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext

import serial
import serial.tools.list_ports

ST_VID = 0x0483       # ST's default USB IDs for the CDC stack (usbd_desc.c)
ST_CDC_PID = 0x5740
POLL_MS = 50          # how often the GUI checks for replies from the worker
ASF_MAX = 0x3FFF      # AD9910 amplitude scale factor is 14 bits
POW_FULL = 65536      # AD9910 phase offset word is 16 bits (one full turn)


# =============================================================================
# SerialWorker : owns the serial port, runs in its own thread (unchanged)
# =============================================================================
class SerialWorker(threading.Thread):
    """Sends one command at a time and waits for its one-line reply."""

    def __init__(self, port_name, results):
        super().__init__(daemon=True)
        # timeout=2: DDS INIT resets the chip and waits for PLL lock, which
        # takes a few ms on the board; 2 s leaves a large margin.
        self.ser = serial.Serial(port_name, timeout=2)
        self.ser.reset_input_buffer()
        self.commands = queue.Queue()
        self.results = results

    def send(self, text):
        self.commands.put(text)

    def stop(self):
        self.commands.put(None)

    def run(self):
        try:
            while True:
                text = self.commands.get()
                if text is None:
                    break
                self.ser.write((text + "\n").encode())
                raw = self.ser.readline()
                reply = raw.decode(errors="replace").rstrip("\r\n")
                self.results.put(("reply", self, text, reply))
        except serial.SerialException as e:
            self.results.put(("error", self, str(e), None))
        finally:
            self.ser.close()


# =============================================================================
# Reply parsing and number formatting
# =============================================================================
def parse_reply(reply):
    """Split a reply line into its parts.

    'OK CHIRP P 1 STEP 17180 DRYRUN' -> (True, 'CHIRP', {'P': '1', 'STEP': '17180'}, True)
    'ERR TONE PARAM'                 -> (False, 'TONE', {'reason': 'PARAM'}, False)
    """
    words = reply.split()
    if len(words) < 2:
        return False, "", {"reason": reply or "no reply"}, False
    ok = words[0] == "OK"
    cmd = words[1]
    rest = words[2:]
    dry = "DRYRUN" in rest
    rest = [w for w in rest if w != "DRYRUN"]
    if ok:
        fields = dict(zip(rest[0::2], rest[1::2]))   # pair up KEY VALUE KEY VALUE
    else:
        fields = {"reason": " ".join(rest)}
    return ok, cmd, fields, dry


def fmt_hz(hz, decimals=3):
    """100000000.093 -> '100 000 000.093 Hz' (space-grouped for readability)."""
    return f"{hz:,.{decimals}f}".replace(",", " ") + " Hz"


def read_number(entry, name):
    """Read a float from an Entry, with a readable error if it is not one."""
    text = entry.get().strip()
    try:
        return float(text)
    except ValueError:
        raise ValueError(f"{name}: '{text}' is not a number")


# =============================================================================
# App : the main window
# =============================================================================
class App(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("RaptorWatch processing board")
        self.minsize(720, 640)

        self.worker = None
        self.results = queue.Queue()
        self.port_map = {}

        # What we know about the AD9910, learned from replies.
        # fsys_hz is None until DDS INIT (or STATUS) reports it.
        self.fsys_hz = None
        self.dds_mode = "NONE"

        # Which reply handler deals with which command word
        self.handlers = {
            "DDS": self.on_dds_reply,
            "TONE": self.on_tone_reply,
            "CHIRP": self.on_chirp_reply,
            "TRAIN": self.on_train_reply,
            "BURST": self.on_train_reply,
            "STOP": self.on_stop_reply,
        }

        self._build_widgets()
        self.refresh_ports()
        self.set_connected(False)
        self.after(POLL_MS, self.poll_results)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # =========================================================================
    # Layout
    # =========================================================================
    def _build_widgets(self):
        pad = {"padx": 6, "pady": 4}

        # ---- Connection row (top, always visible) ---------------------------
        conn = ttk.LabelFrame(self, text="Connection")
        conn.pack(fill="x", **pad)
        ttk.Label(conn, text="Port:").pack(side="left", **pad)
        self.port_box = ttk.Combobox(conn, state="readonly", width=38)
        self.port_box.pack(side="left", **pad)
        self.refresh_btn = ttk.Button(conn, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.pack(side="left", **pad)
        self.connect_btn = ttk.Button(conn, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", **pad)
        self.status = ttk.Label(conn, text="Disconnected", foreground="grey")
        self.status.pack(side="left", **pad)

        # ---- Tabs ------------------------------------------------------------
        # A Notebook is a tabbed container; each tab is an ordinary Frame.
        tabs = ttk.Notebook(self)
        tabs.pack(fill="x", **pad)
        board_tab = ttk.Frame(tabs)
        dds_tab = ttk.Frame(tabs)
        tabs.add(dds_tab, text="DDS")
        tabs.add(board_tab, text="Board")

        self._build_board_tab(board_tab, pad)
        self._build_dds_tab(dds_tab, pad)

        # ---- Log (bottom, always visible) ------------------------------------
        logf = ttk.LabelFrame(self, text="Log")
        logf.pack(fill="both", expand=True, **pad)
        self.log_box = scrolledtext.ScrolledText(logf, height=10, state="disabled",
                                                 font=("Consolas", 10))
        self.log_box.pack(fill="both", expand=True, **pad)
        ttk.Button(logf, text="Clear log", command=self.clear_log).pack(anchor="e", **pad)

    def _build_board_tab(self, parent, pad):
        leds = ttk.LabelFrame(parent, text="LEDs")
        leds.pack(fill="x", **pad)
        self.led_vars = [tk.BooleanVar() for _ in range(3)]
        self.led_checks = []
        for var, label in zip(self.led_vars,
                              ["LED0 (red, PC13)", "LED1 (green, PC14)", "LED2 (blue, PC15)"]):
            cb = ttk.Checkbutton(leds, text=label, variable=var, command=self.on_led_change)
            cb.pack(side="left", **pad)
            self.led_checks.append(cb)

        cmds = ttk.LabelFrame(parent, text="Raw command")
        cmds.pack(fill="x", **pad)
        self.ping_btn = ttk.Button(cmds, text="PING", command=lambda: self.send_command("PING"))
        self.ping_btn.pack(side="left", **pad)
        self.stat_btn = ttk.Button(cmds, text="STAT", command=lambda: self.send_command("STAT"))
        self.stat_btn.pack(side="left", **pad)
        self.entry = ttk.Entry(cmds)
        self.entry.pack(side="left", fill="x", expand=True, **pad)
        self.entry.bind("<Return>", lambda e: self.send_entry())
        self.send_btn = ttk.Button(cmds, text="Send", command=self.send_entry)
        self.send_btn.pack(side="left", **pad)

    def _build_dds_tab(self, parent, pad):
        # ---- Device: init / status / reset ----------------------------------
        dev = ttk.LabelFrame(parent, text="AD9910")
        dev.pack(fill="x", **pad)
        self.init_btn = ttk.Button(dev, text="Init", command=lambda: self.send_command("DDS INIT"))
        self.init_btn.grid(row=0, column=0, **pad)
        self.dstat_btn = ttk.Button(dev, text="Status", command=lambda: self.send_command("DDS STATUS"))
        self.dstat_btn.grid(row=0, column=1, **pad)
        self.reset_btn = ttk.Button(dev, text="Reset", command=lambda: self.send_command("DDS RESET"))
        self.reset_btn.grid(row=0, column=2, **pad)

        self.lbl_init = ttk.Label(dev, text="Init: ?")
        self.lbl_init.grid(row=0, column=3, sticky="w", **pad)
        self.lbl_fsys = ttk.Label(dev, text="f_SYSCLK: ?")
        self.lbl_fsys.grid(row=0, column=4, sticky="w", **pad)
        self.lbl_mode = ttk.Label(dev, text="Mode: ?")
        self.lbl_mode.grid(row=0, column=5, sticky="w", **pad)
        self.lbl_lock = ttk.Label(dev, text="PLL lock: ?")
        self.lbl_lock.grid(row=0, column=6, sticky="w", **pad)
        # Dry-run badge: makes it impossible to mistake simulated replies for
        # real hardware results.
        self.lbl_dry = ttk.Label(dev, text="", foreground="orange")
        self.lbl_dry.grid(row=1, column=0, columnspan=7, sticky="w", **pad)

        # ---- Single tone ------------------------------------------------------
        tone = ttk.LabelFrame(parent, text="Single tone")
        tone.pack(fill="x", **pad)
        self.tone_f = self._labelled_entry(tone, "Frequency [MHz]", "100", 0, 0, pad)
        self.tone_a = self._labelled_entry(tone, "Amplitude [% FS]", "100", 0, 2, pad)
        self.tone_p = self._labelled_entry(tone, "Phase [deg]", "0", 0, 4, pad)
        self.tone_btn = ttk.Button(tone, text="Apply tone", command=self.apply_tone)
        self.tone_btn.grid(row=0, column=6, **pad)
        for e in (self.tone_f, self.tone_a, self.tone_p):
            e.bind("<Return>", lambda ev: self.apply_tone())
        self.tone_result = ttk.Label(tone, text="", font=("Consolas", 10))
        self.tone_result.grid(row=1, column=0, columnspan=7, sticky="w", **pad)

        # ---- Chirp ------------------------------------------------------------
        chirp = ttk.LabelFrame(parent, text="Linear chirp (digital ramp generator)")
        chirp.pack(fill="x", **pad)
        self.ch_f0 = self._labelled_entry(chirp, "Start [MHz]", "100", 0, 0, pad)
        self.ch_f1 = self._labelled_entry(chirp, "Stop [MHz]", "200", 0, 2, pad)
        self.ch_t = self._labelled_entry(chirp, "Sweep time [us]", "100", 0, 4, pad)
        for e in (self.ch_f0, self.ch_f1, self.ch_t):
            e.bind("<Return>", lambda ev: self.load_chirp())

        # Radio buttons share one StringVar; selecting one sets its value.
        self.ch_mode = tk.StringVar(value="SAW")
        modes = ttk.Frame(chirp)
        modes.grid(row=1, column=0, columnspan=4, sticky="w", **pad)
        ttk.Label(modes, text="Mode:").pack(side="left")
        self.saw_rb = ttk.Radiobutton(modes, text="Sawtooth (triggered)",
                                      variable=self.ch_mode, value="SAW")
        self.saw_rb.pack(side="left", padx=6)
        self.tri_rb = ttk.Radiobutton(modes, text="Triangle (free-running)",
                                      variable=self.ch_mode, value="TRI")
        self.tri_rb.pack(side="left", padx=6)

        btns = ttk.Frame(chirp)
        btns.grid(row=1, column=4, columnspan=3, sticky="e", **pad)
        self.chirp_btn = ttk.Button(btns, text="Load chirp", command=self.load_chirp)
        self.chirp_btn.pack(side="left", padx=4)
        self.trig_btn = ttk.Button(btns, text="Trigger",
                                   command=lambda: self.send_command("TRIG"))
        self.trig_btn.pack(side="left", padx=4)
        self.hold_var = tk.BooleanVar()
        self.hold_cb = ttk.Checkbutton(btns, text="Hold", variable=self.hold_var,
                                       command=self.on_hold_change)
        self.hold_cb.pack(side="left", padx=4)

        # Multi-line readout of what the board actually programmed
        self.chirp_result = ttk.Label(chirp, text="", font=("Consolas", 10), justify="left")
        self.chirp_result.grid(row=2, column=0, columnspan=7, sticky="w", **pad)

        # ---- Chirp train (TIM1 drives DRCTL) ----------------------------------
        # The period and count are sent once; the STM32's TIM1 then produces
        # the DRCTL edges in hardware, so chirp spacing does not depend on
        # USB or Windows timing.
        train = ttk.LabelFrame(parent, text="Chirp train (hardware-timed by TIM1 on DRCTL)")
        train.pack(fill="x", **pad)
        self.tr_period = self._labelled_entry(train, "Period [us]", "200", 0, 0, pad)
        self.tr_count = self._labelled_entry(train, "Chirps (0 = continuous)", "64", 0, 2, pad)
        for e in (self.tr_period, self.tr_count):
            e.bind("<Return>", lambda ev: self.start_train())
        tbtns = ttk.Frame(train)
        tbtns.grid(row=0, column=4, sticky="e", **pad)
        self.train_btn = ttk.Button(tbtns, text="Start", command=self.start_train)
        self.train_btn.pack(side="left", padx=4)
        self.stop_btn = ttk.Button(tbtns, text="Stop", command=lambda: self.send_command("STOP"))
        self.stop_btn.pack(side="left", padx=4)
        self.lbl_train = ttk.Label(tbtns, text="Train: ?")
        self.lbl_train.pack(side="left", padx=8)
        self.train_result = ttk.Label(train, text="", font=("Consolas", 10), justify="left")
        self.train_result.grid(row=1, column=0, columnspan=5, sticky="w", **pad)

        # Controls enabled only while connected (fine-grained rules for the
        # DDS ones live in update_dds_controls)
        self.connected_only = [*self.led_checks, self.ping_btn, self.stat_btn,
                               self.entry, self.send_btn,
                               self.init_btn, self.dstat_btn, self.reset_btn]

    def _labelled_entry(self, parent, label, default, row, col, pad):
        """Label + Entry pair in a grid; returns the Entry."""
        ttk.Label(parent, text=label).grid(row=row, column=col, sticky="e", **pad)
        e = ttk.Entry(parent, width=12)
        e.insert(0, default)
        e.grid(row=row, column=col + 1, sticky="w", **pad)
        return e

    # =========================================================================
    # Connection handling (unchanged apart from the STATUS query on connect)
    # =========================================================================
    def refresh_ports(self):
        self.port_map.clear()
        preferred = None
        for p in serial.tools.list_ports.comports():
            label = f"{p.device}  ({p.description})"
            self.port_map[label] = p.device
            if p.vid == ST_VID and p.pid == ST_CDC_PID:
                preferred = label
        labels = list(self.port_map)
        self.port_box["values"] = labels
        self.port_box.set(preferred or (labels[0] if labels else ""))

    def toggle_connect(self):
        if self.worker:
            self.disconnect()
            return
        device = self.port_map.get(self.port_box.get())
        if not device:
            self.log("! No port selected")
            return
        try:
            self.worker = SerialWorker(device, self.results)
        except serial.SerialException as e:
            self.log(f"! Could not open {device}: {e}")
            return
        self.worker.start()
        self.set_connected(True)
        self.log(f"Opened {device}")
        self.send_command("PING")
        # Ask the firmware what state the DDS is in, so the GUI starts in
        # sync with the board even if the GUI was restarted mid-session.
        self.send_command("DDS STATUS")

    def disconnect(self):
        if self.worker:
            self.worker.stop()
            self.worker = None
        self.set_connected(False)
        self.log("Disconnected")

    def set_connected(self, on):
        state = ["!disabled"] if on else ["disabled"]
        for w in self.connected_only:
            w.state(state)
        self.port_box.state(["disabled"] if on else ["!disabled", "readonly"])
        self.refresh_btn.state(["disabled"] if on else ["!disabled"])
        self.connect_btn.config(text="Disconnect" if on else "Connect")
        if on:
            self.status.config(text="Port open, waiting for PONG...", foreground="orange")
        else:
            self.status.config(text="Disconnected", foreground="grey")
            # Unknown until we hear from the board again
            self.fsys_hz = None
            self.dds_mode = "NONE"
            self.refresh_dds_labels(init=None, lock="?", dry=False)
        self.update_dds_controls()

    # =========================================================================
    # DDS control enabling
    # =========================================================================
    def update_dds_controls(self):
        """Enable only the DDS actions that make sense in the current state.
        The firmware enforces the same rules; doing it here too means the user
        cannot even click something that would be refused."""
        connected = self.worker is not None
        ready = connected and self.fsys_hz is not None      # DDS initialised
        chirp_loaded = self.dds_mode in ("SAW", "TRI")

        def en(widget, cond):
            widget.state(["!disabled"] if cond else ["disabled"])

        en(self.tone_btn, ready)
        en(self.chirp_btn, ready)
        en(self.saw_rb, ready)
        en(self.tri_rb, ready)
        en(self.trig_btn, ready and self.dds_mode == "SAW")  # triggers only mean something for SAW
        en(self.hold_cb, ready and chirp_loaded)
        en(self.train_btn, ready and self.dds_mode == "SAW")  # trains trigger sawtooth chirps
        en(self.stop_btn, ready)
        if not chirp_loaded:
            self.hold_var.set(False)

    def refresh_dds_labels(self, init, lock, dry):
        self.lbl_init.config(text=f"Init: {'?' if init is None else ('yes' if init else 'no')}",
                             foreground="green" if init else "black")
        self.lbl_fsys.config(text="f_SYSCLK: ?" if self.fsys_hz is None
                             else f"f_SYSCLK: {self.fsys_hz / 1e6:.3f} MHz")
        self.lbl_mode.config(text=f"Mode: {self.dds_mode}")
        self.lbl_lock.config(text=f"PLL lock: {lock}")
        self.lbl_dry.config(text="DRY RUN: firmware is not driving the AD9910, "
                                 "values below are computed only" if dry else "")

    # =========================================================================
    # Sending: build firmware commands from the GUI fields
    # =========================================================================
    def send_command(self, text):
        if not self.worker:
            return
        self.log(f"> {text}")
        self.worker.send(text)

    def send_entry(self):
        text = self.entry.get().strip()
        if text:
            self.send_command(text)
            self.entry.delete(0, "end")

    def on_led_change(self):
        mask = sum(1 << bit for bit, var in enumerate(self.led_vars) if var.get())
        self.send_command(f"LED {mask}")

    def apply_tone(self):
        """MHz / % / degrees -> TONE <f_hz> <asf> <pow>"""
        try:
            f_mhz = read_number(self.tone_f, "Frequency")
            a_pct = read_number(self.tone_a, "Amplitude")
            p_deg = read_number(self.tone_p, "Phase")
            if f_mhz <= 0:
                raise ValueError("Frequency must be positive")
            if not 0 <= a_pct <= 100:
                raise ValueError("Amplitude must be 0 to 100 %")
        except ValueError as e:
            self.show_input_error(self.tone_result, e)
            return

        f_hz = round(f_mhz * 1e6)                  # integer Hz for the link
        asf = round(a_pct / 100 * ASF_MAX)         # 100 % -> 16383
        # Phase wraps: 360 deg is one full turn = 65536 POW counts.
        pow_word = round((p_deg % 360) / 360 * POW_FULL) % POW_FULL
        self.send_command(f"TONE {f_hz} {asf} {pow_word}")

    def load_chirp(self):
        """MHz / us -> CHIRP <f0_hz> <f1_hz> <t_ns> <SAW|TRI>"""
        try:
            f0 = read_number(self.ch_f0, "Start")
            f1 = read_number(self.ch_f1, "Stop")
            t_us = read_number(self.ch_t, "Sweep time")
            if f0 < 0 or f1 <= f0:
                raise ValueError("Stop must be above start, and start must be >= 0")
            t_ns = round(t_us * 1000)
            if t_ns < 1:
                raise ValueError("Sweep time must be at least 1 ns")
        except ValueError as e:
            self.show_input_error(self.chirp_result, e)
            return
        # Range limits that depend on f_SYSCLK (0.4 fs, DRG timer range) are
        # checked by the firmware, which knows the real clock.
        self.send_command(f"CHIRP {round(f0 * 1e6)} {round(f1 * 1e6)} {t_ns} {self.ch_mode.get()}")

    def start_train(self):
        """us / count -> TRAIN <period_ns>  or  BURST <n> <period_ns>"""
        try:
            p_us = read_number(self.tr_period, "Period")
            count = read_number(self.tr_count, "Chirps")
            if p_us <= 0:
                raise ValueError("Period must be positive")
            if count < 0 or count != int(count):
                raise ValueError("Chirps must be a whole number >= 0")
        except ValueError as e:
            self.show_input_error(self.train_result, e)
            return
        period_ns = round(p_us * 1000)
        if int(count) == 0:
            self.send_command(f"TRAIN {period_ns}")
        else:
            self.send_command(f"BURST {int(count)} {period_ns}")

    def on_hold_change(self):
        self.send_command(f"HOLD {1 if self.hold_var.get() else 0}")

    def show_input_error(self, label, err):
        label.config(text=f"Input error: {err}", foreground="red")
        self.log(f"! {err}")

    # =========================================================================
    # Receiving: route each reply to a handler
    # =========================================================================
    def poll_results(self):
        while True:
            try:
                kind, worker, a, b = self.results.get_nowait()
            except queue.Empty:
                break
            if worker is not self.worker:
                continue

            if kind == "reply":
                self.handle_reply(a, b)
            elif kind == "error":
                self.log(f"! Serial error: {a}")
                self.worker = None
                self.set_connected(False)

        self.after(POLL_MS, self.poll_results)

    def handle_reply(self, cmd, reply):
        if reply == "":
            self.log("< (no reply within 2 s)")
            if cmd == "PING":
                self.status.config(text="No response from board", foreground="red")
            return

        self.log(f"< {reply}")
        if cmd == "PING" and reply == "PONG":
            self.status.config(text="Connected, board responding", foreground="green")
            return

        ok, word, fields, dry = parse_reply(reply)
        handler = self.handlers.get(cmd.split()[0])   # route by the command we SENT
        if handler:
            handler(cmd, ok, fields, dry)
        self.update_dds_controls()

    # ---- DDS INIT / STATUS / RESET ------------------------------------------
    def on_dds_reply(self, cmd, ok, fields, dry):
        sub = cmd.split()[1] if len(cmd.split()) > 1 else ""
        if not ok:
            if sub == "INIT":
                self.fsys_hz = None           # a failed init leaves the chip unconfigured
                self.dds_mode = "NONE"
                self.lbl_init.config(text=f"Init failed: {fields['reason']}", foreground="red")
            return

        lock = "?"
        init = None
        if sub == "INIT":
            self.fsys_hz = int(fields["FSYS"])
            self.dds_mode = "NONE"
            init = True
            self.set_train_label("IDLE")     # firmware stops the train on INIT
        elif sub == "STATUS":
            init = fields.get("INIT") == "1"
            self.fsys_hz = int(fields["FSYS"]) if init else None
            self.dds_mode = fields.get("MODE", "NONE")
            lock = fields.get("LOCK", "?")
            self.set_train_label(fields.get("TRAIN", "?"))
        elif sub == "RESET":
            self.fsys_hz = None
            self.dds_mode = "NONE"
            init = False
            self.set_train_label("IDLE")
            self.train_result.config(text="")
            self.tone_result.config(text="")
            self.chirp_result.config(text="")
        self.refresh_dds_labels(init, lock, dry)

    # ---- TONE -----------------------------------------------------------------
    def on_tone_reply(self, cmd, ok, fields, dry):
        if not ok:
            self.tone_result.config(text=f"Refused: {fields['reason']}", foreground="red")
            return
        f_req = int(cmd.split()[1])                # what we asked for (from our own command)
        f_act = float(fields["FACT"])              # what the FTW really produces
        asf = int(fields["ASF"])
        self.tone_result.config(
            foreground="black",
            text=(f"FTW {fields['FTW']}   actual {fmt_hz(f_act)} "
                  f"({f_act - f_req:+.3f} Hz)   amplitude {100 * asf / ASF_MAX:.2f} % FS"))
        self.dds_mode = "TONE"
        self.lbl_mode.config(text="Mode: TONE")
        self.set_train_label("IDLE")         # firmware stops the train on TONE

    # ---- CHIRP ----------------------------------------------------------------
    def on_chirp_reply(self, cmd, ok, fields, dry):
        if not ok:
            self.chirp_result.config(text=f"Refused: {fields['reason']}", foreground="red")
            return

        # Requested values, recovered from the command we sent
        _, f0, f1, t_ns, mode = cmd.split()
        f0, f1, t_req = int(f0), int(f1), int(t_ns) * 1e-9
        slope_req = (f1 - f0) / t_req

        # What the firmware actually programmed after integer rounding
        P = int(fields["P"])
        step = int(fields["STEP"])
        n_steps = int(fields["NSTEPS"])
        t_act = int(fields["T_NS"]) * 1e-9
        slope_act = int(fields["SLOPE"])

        lines = [
            f"Sweep time : {t_act * 1e6:.3f} us   (requested {t_req * 1e6:.3f} us)",
            f"Slope      : {slope_act / 1e12:.6f} MHz/us   "
            f"(error {1e6 * (slope_act - slope_req) / slope_req:+.1f} ppm)",
            f"Steps      : {n_steps} x STEP {step}, rate word P = {P}",
        ]
        if self.fsys_hz:
            # Physical meaning of P and STEP (see driver explainer, section 9)
            dt_ns = 4 * P / self.fsys_hz * 1e9
            df_hz = step * self.fsys_hz / 2**32
            lines.append(f"Staircase  : {df_hz:,.1f} Hz every {dt_ns:.1f} ns".replace(",", " "))
        self.chirp_result.config(text="\n".join(lines), foreground="black")

        self.dds_mode = mode
        self.lbl_mode.config(text=f"Mode: {mode}")
        self.set_train_label("IDLE")         # firmware stops the train on CHIRP

    # ---- TRAIN / BURST / STOP ---------------------------------------------------
    def set_train_label(self, state):
        colours = {"RUN": "green", "BURST": "green", "IDLE": "black"}
        self.lbl_train.config(text=f"Train: {state}", foreground=colours.get(state, "black"))

    def on_train_reply(self, cmd, ok, fields, dry):
        if not ok:
            self.train_result.config(text=f"Refused: {fields['reason']}", foreground="red")
            return

        words = cmd.split()
        req_ns = int(words[-1])                     # requested period (last word)
        act_ns = float(fields["PERIOD_NS"])         # achieved period after tick rounding
        psc = int(fields["PSC"])
        arr = int(fields["ARR"])
        ccr = int(fields["CCR1"])
        tick_ns = 12.5 * (psc + 1)                  # counter tick at 80 MHz / (PSC+1)
        pulse_ns = (arr + 1 - ccr) * tick_ns        # DRCTL high time per chirp

        lines = [
            f"Period : {act_ns / 1000:.4f} us   (requested {req_ns / 1000:.4f} us, "
            f"error {act_ns - req_ns:+.1f} ns)   = {1e9 / act_ns:.1f} chirps/s",
            f"TIM1   : PSC {psc}  ARR {arr}  CCR1 {ccr}   tick {tick_ns:.1f} ns, "
            f"DRCTL pulse {pulse_ns:.1f} ns",
        ]
        if words[0] == "BURST":
            n = int(fields["N"])
            dur_ms = n * act_ns / 1e6
            lines.append(f"Burst  : {n} chirps, RCR {fields['RCR']}, lasts {dur_ms:.3f} ms")
            self.set_train_label("BURST")
            # The timer stops itself after N periods; ask the board afterwards
            # so the label returns to IDLE. (Skipped for very long bursts.)
            if dur_ms < 60000:
                self.after(int(dur_ms) + 100, lambda: self.send_command("DDS STATUS"))
        else:
            lines.append("Mode   : continuous until Stop")
            self.set_train_label("RUN")
        self.train_result.config(text="\n".join(lines), foreground="black")

    def on_stop_reply(self, cmd, ok, fields, dry):
        if ok:
            self.set_train_label("IDLE")

    # =========================================================================
    # Log helpers
    # =========================================================================
    def log(self, msg):
        stamp = time.strftime("%H:%M:%S")
        self.log_box.config(state="normal")
        self.log_box.insert("end", f"{stamp}  {msg}\n")
        self.log_box.see("end")
        self.log_box.config(state="disabled")

    def clear_log(self):
        self.log_box.config(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.config(state="disabled")

    def on_close(self):
        if self.worker:
            self.worker.stop()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()