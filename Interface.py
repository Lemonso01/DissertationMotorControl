# motor_gui.py

import tkinter as tk
from tkinter import ttk, messagebox
import threading, time

# ──────────────────────────────────────────────────────────────
# 1) Serial fallback logic
# ──────────────────────────────────────────────────────────────
try:
    import serial
    REAL_SERIAL = True
except ImportError:
    REAL_SERIAL = False

class DummySerial:
    def __init__(self, *args, **kwargs):
        self._in = b""
    def write(self, data):
        self._in += b"ECHO: " + data
    def read(self, n=1):
        if not self._in:
            time.sleep(0.05)
            return b""
        out, self._in = self._in, b""
        return out

# ──────────────────────────────────────────────────────────────
# 2) Configuration
# ──────────────────────────────────────────────────────────────
SERIAL_PORT = "COM6"    # adjust as needed
BAUDRATE    = 115200
MOTOR_ID    = 1         # your CAN node ID

# ──────────────────────────────────────────────────────────────
# 3) GUI Application
# ──────────────────────────────────────────────────────────────
class CanGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Motor Control GUI")

        # Style for STOP button
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("Red.TButton",
            foreground="white",
            background="red",
            font=("Segoe UI", 10, "bold"),
            padding=6,
            borderwidth=1)
        style.map("Red.TButton",
            foreground=[("!disabled", "white")],
            background=[
                ("!disabled", "red"),
                ("active",   "firebrick"),
                ("pressed",  "darkred")
            ])

        # Serial port (or dummy)
        try:
            if REAL_SERIAL:
                self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
            else:
                raise Exception("pyserial missing or port not found")
        except Exception as e:
            messagebox.showwarning("Serial Port", f"Falling back to dummy:\n{e}")
            self.ser = DummySerial()

        # auto-enter MIT mode
        self.after(500, lambda: self.send_cmd("MITPING"))

        # MIT parameters defaults (Position, Velocity, Kp, Kd, Torque feed-forward)
        self.exec_params = {
            'p':   1.5708,   # rad
            'v':   0.0,      # rad/s
            'kp': 100.0,
            'kd':   1.0,
            'tff':  0.0,     # N·m
        }

        # Tab notebook
        nb = ttk.Notebook(self)
        nb.grid(row=0, column=0, padx=10, pady=10)

        # ── Main tab ──
        main_f = ttk.Frame(nb, padding=20)
        nb.add(main_f, text="Main")

        ttk.Button(main_f, text="Calibration",
                   command=self.do_calibrate, width=20)\
            .pack(pady=(0,10))

        exec_frame = ttk.Frame(main_f)
        exec_frame.pack(pady=(0,10))
        ttk.Button(exec_frame, text="Execute Move",
                   command=self.do_execute, width=14)\
            .grid(row=0, column=0)
        ttk.Button(exec_frame, text="⚙", width=4,
                   command=self.open_exec_dialog)\
            .grid(row=0, column=1, padx=(5,0))

        # Resistance Mode toggle
        self.resist_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(main_f, text="Resistance Mode",
                        variable=self.resist_var,
                        command=self.toggle_resistance)\
            .pack(pady=(10,0))

        ttk.Button(main_f, text="STOP",
                   command=self.do_stop,
                   width=20, style="Red.TButton")\
            .pack()

        # ── Advanced tab ──
        adv_f = ttk.Frame(nb, padding=10)
        nb.add(adv_f, text="Advanced")

        # Servo POS / RPM
        ttk.Label(adv_f, text="Servo POS (°):")\
            .grid(row=0, column=0, sticky="e")
        self.pos_var = tk.DoubleVar(value=90.0)
        ttk.Entry(adv_f, textvariable=self.pos_var, width=8)\
            .grid(row=0, column=1)
        ttk.Button(adv_f, text="GO",
                   command=self.do_servo_pos)\
            .grid(row=0, column=2, padx=5)

        ttk.Label(adv_f, text="Servo RPM:")\
            .grid(row=1, column=0, sticky="e")
        self.rpm_var = tk.IntVar(value=1000)
        ttk.Entry(adv_f, textvariable=self.rpm_var, width=8)\
            .grid(row=1, column=1)
        ttk.Button(adv_f, text="SET",
                   command=self.do_servo_rpm)\
            .grid(row=1, column=2, padx=5)

        ttk.Separator(adv_f, orient="horizontal")\
            .grid(row=2, columnspan=3, sticky="ew", pady=5)

        # MIT controls
        ttk.Button(adv_f, text="MIT ENTER",
                   command=lambda: self.send_cmd("MITPING"))\
            .grid(row=3, column=0, columnspan=3, sticky="ew", pady=(0,5))

        ttk.Label(adv_f, text="MIT POS (rad):")\
            .grid(row=4, column=0, sticky="e")
        self.mit_var = tk.DoubleVar(value=1.5708)
        ttk.Entry(adv_f, textvariable=self.mit_var, width=8)\
            .grid(row=4, column=1)
        ttk.Button(adv_f, text="MIT GO",
                   command=self.do_mitpos)\
            .grid(row=4, column=2, padx=5)

        ttk.Button(adv_f, text="SCAN IDs",
                   command=lambda: self.send_cmd("SCAN"))\
            .grid(row=5, column=0, columnspan=3, sticky="ew", pady=(5,0))

        # Log pane
        self.log = tk.Text(self, width=50, height=12,
                           state="disabled", takefocus=False)
        self.log.grid(row=1, column=0, padx=10, pady=(0,10))

        # Start reader thread
        threading.Thread(target=self.reader, daemon=True).start()

    # ─── Main tab handlers ──────────────────────────────────────
    def do_calibrate(self):
        self.send_cmd("CALIBRATE")

    def open_exec_dialog(self):
        dlg = tk.Toplevel(self)
        dlg.title("Setup Execute Parameters")
        dlg.resizable(False, False)

        labels = ["Position (rad):", "Velocity (rad/s):", "Kp:", "Kd:", "Torque (N·m):"]
        keys   = ['p','v','kp','kd','tff']
        entries = {}

        for i,(lbl,key) in enumerate(zip(labels,keys)):
            ttk.Label(dlg, text=lbl)\
                .grid(row=i, column=0, sticky="e", pady=2, padx=5)
            var = tk.DoubleVar(value=self.exec_params[key])
            ttk.Entry(dlg, textvariable=var, width=12)\
                .grid(row=i, column=1, pady=2)
            entries[key] = var

        def on_ok():
            for key,var in entries.items():
                self.exec_params[key] = var.get()
            dlg.destroy()

        ttk.Button(dlg, text="OK",
                   command=on_ok)\
            .grid(row=len(labels), column=0, pady=10)
        ttk.Button(dlg, text="Cancel",
                   command=dlg.destroy)\
            .grid(row=len(labels), column=1, pady=10)

    def do_execute(self):
        p   = self.exec_params['p']
        v   = self.exec_params['v']
        kp  = self.exec_params['kp']
        kd  = self.exec_params['kd']
        tff = self.exec_params['tff']
        cmd = f"MITCMD {p:.4f} {v:.4f} {kp:.2f} {kd:.2f} {tff:.2f}"
        self.send_cmd(cmd)

    def toggle_resistance(self):
        if self.resist_var.get():
            # Zero out loops, only torque remains
            v   = 0.0
            kp  = 0.0
            kd  = 0.0
            tff = self.exec_params.get('tff', 1.0)
            self.send_cmd(f"MITCMD 0.0000 {v:.4f} {kp:.2f} {kd:.2f} {tff:.2f}")
            self.log_write(f"> Resistance ON: tff={tff:.2f}Nm")
        else:
            # Restore loops, zero torque
            v   = self.exec_params['v']
            kp  = self.exec_params['kp']
            kd  = self.exec_params['kd']
            self.send_cmd(f"MITCMD 0.0000 {v:.4f} {kp:.2f} {kd:.2f} 0.00")
            self.log_write("> Resistance OFF")

    def do_stop(self):
        self.send_cmd("RPM 0")
        self.send_cmd("MITCMD 0.0000 0.0000 0.00 0.00 0.00")

    # ─── Advanced tab handlers ──────────────────────────────────
    def do_servo_pos(self):
        self.send_cmd(f"POS {self.pos_var.get():.1f}")

    def do_servo_rpm(self):
        self.send_cmd(f"RPM {self.rpm_var.get()}")

    def do_mitpos(self):
        self.send_cmd(f"MITPOS {self.mit_var.get():.4f}")

    # ─── Serial & Logging ───────────────────────────────────────
    def send_cmd(self, cmd):
        self.ser.write((cmd + "\n").encode())
        self.log_write(f"> {cmd}")

    def log_write(self, txt):
        self.log.configure(state="normal")
        self.log.insert("end", txt + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def reader(self):
        buf = b""
        while True:
            data = self.ser.read(128)
            if data:
                buf += data
                lines = buf.split(b"\n")
                for ln in lines[:-1]:
                    try:
                        text = ln.decode('utf-8', errors='ignore')
                    except:
                        text = repr(ln)
                    self.log_write(text)
                buf = lines[-1]
            time.sleep(0.05)

if __name__ == "__main__":
    CanGui().mainloop()
