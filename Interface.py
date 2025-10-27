# motor_gui.py
# GUI for serial motor control + real-time dual-motor status and graphics
# Graphics:
#  - Elbow: upper arm fixed LEFT, forearm rotates; 0° = right (extension), +deg = CCW.
#  - Wrist dial: −90°..+90°, numeric 0° points RIGHT (drawing offset applied).
#
# Simulation:
#  - "Simulate Move" button opens a dialog to pick target angles (Elbow/Wrist).
#  - On Start, graphics move toward targets at 10 rpm (60 deg/s) without affecting serial IO.

import threading
import time
import re
import math
import serial
from serial.tools import list_ports

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    REAL_TKINTER = True
except ImportError:
    REAL_TKINTER = False

# ---------------- Configuration ----------------
SERIAL_PORT = "COM3"
BAUDRATE    = 115200

# MCU streams status continuously (no polling)
STATUS_POLL_ENABLED    = False
STATUS_POLL_PERIOD_S   = 0.10  # ignored when polling disabled

# Simulation speed
SIM_SPEED_DEG_PER_S    = 60.0   # 10 rpm = 60 deg/s
SIM_TICK_S             = 0.02   # 50 Hz updates

# Torque constants (Nm/A) for each motor (used if torque not provided)
KT_NM_PER_A_1          = 0.08   # Motor 1 (Elbow)
KT_NM_PER_A_2          = 0.08   # Motor 2 (Wrist)

# Drawing sizes
ELBOW_L_UPPER          = 110  # px
ELBOW_L_FOREARM        = 110  # px
WRIST_DIAL_R           = 70   # px
WRIST_DIAL_OFFSET_DEG  = 90   # rotate drawing so numeric 0° points RIGHT visually

class DummySerial:
    def __init__(self, *args, **kwargs): self._in = b""
    def write(self, data): self._in += b"<ECHO> " + data
    def read(self, n=1):
        if not self._in:
            time.sleep(0.05)
            return b""
        out, self._in = self._in, b""
        return out

if REAL_TKINTER:
    class CanGui(tk.Tk):
        def __init__(self):
            super().__init__()
            self.title("Motor Control GUI")

            style = ttk.Style(self)
            style.theme_use("clam")
            style.configure("Red.TButton", foreground="white", background="red")
            style.configure("Blue.TButton", foreground="white", background="blue")
            style.configure("Orange.TButton", foreground="white", background="orange")

            # Serial initialization
            ports = [p.device for p in list_ports.comports()]
            self.using_dummy = False
            for cand in (SERIAL_PORT, f"\\.\\{SERIAL_PORT}"):
                if cand in ports:
                    try:
                        self.ser = serial.Serial(cand, BAUDRATE, timeout=0.05)
                        break
                    except:
                        pass
            else:
                messagebox.showwarning("Serial Port", f"Couldn’t open {SERIAL_PORT}; using dummy.")
                self.ser = DummySerial()
                self.using_dummy = True

            # Parameters (for your command buttons)
            self.params = dict(
                rpm=0.0, pos=0.0, torque=0.0,
                duty=0.0, current=0.0, brake=0.0,
                posspd_p=0.0, posspd_v=0.0, posspd_a=0.0,
                aan_s=0.0, aan_e=0.0, aan_d=5.0
            )

            # Simulation control flags/handles
            self._sim_stop = threading.Event()
            self._sim_thread = None
            self._sim_lock = threading.Lock()

            # ---------- Layout ----------
            main = ttk.Frame(self, padding=10)
            main.pack(fill="both", expand=True)

            left = ttk.Frame(main)
            left.pack(side="left", fill="y", padx=(0,10))

            right = ttk.Frame(main)
            right.pack(side="right", fill="both", expand=True)

            # Left controls
            left.grid_columnconfigure(0, weight=1)
            left.grid_columnconfigure(1, minsize=10)
            left.grid_columnconfigure(2, weight=1)

            ttk.Button(left, text="⚙", width=3, command=self.open_settings).grid(row=0, column=0, columnspan=3, pady=(0,5))
            ttk.Button(left, text="Set Origin", style="Blue.TButton", command=lambda: self.send_cmd("ORIGIN")).grid(row=1, column=0, columnspan=3, sticky="ew", pady=2)
            ttk.Button(left, text="Calibrate", style="Orange.TButton", command=lambda: self.send_cmd("CALIBRATE")).grid(row=2, column=0, columnspan=3, sticky="ew", pady=2)
            ttk.Button(left, text="STOP", style="Red.TButton", command=self.stop_all).grid(row=3, column=0, columnspan=3, sticky="ew", pady=2)

            ttk.Label(left, text="TEST MODES", font=(None,10,'bold')).grid(row=4, column=0, pady=(10,2))
            ttk.Separator(left, orient='vertical').grid(row=4, column=1, rowspan=9, sticky='ns')
            ttk.Label(left, text="WORK MODES", font=(None,10,'bold')).grid(row=4, column=2, pady=(10,2))

            ttk.Button(left, text="RPM",    command=lambda: self.send_cmd(f"RPM {int(self.params['rpm'])}")).grid(row=5, column=0, sticky='ew', pady=2)
            ttk.Button(left, text="POS",    command=lambda: self.send_cmd(f"POS {self.params['pos']:.2f}" )).grid(row=6, column=0, sticky='ew', pady=2)
            ttk.Button(left, text="TORQUE", command=lambda: self.send_cmd(f"TORQUE {self.params['torque']:.2f}" )).grid(row=7, column=0, sticky='ew', pady=2)
            ttk.Button(left, text="DUTY",   command=lambda: self.send_cmd(f"DUTY {self.params['duty']:.3f}" )).grid(row=8, column=0, sticky='ew', pady=2)
            ttk.Button(left, text="CURRENT",command=lambda: self.send_cmd(f"CURRENT {self.params['current']:.2f}" )).grid(row=9, column=0, sticky='ew', pady=2)
            ttk.Button(left, text="BRAKE",  command=lambda: self.send_cmd(f"BRAKE {self.params['brake']:.2f}" )).grid(row=10, column=0, sticky='ew', pady=2)
            ttk.Button(left, text="POSSPD", command=lambda: self.send_cmd(
                f"POSSPD {self.params['posspd_p']:.2f} {self.params['posspd_v']:.2f} {self.params['posspd_a']:.2f}" )
            ).grid(row=11, column=0, sticky='ew', pady=2)

            ttk.Button(left, text="Assist A/N", command=self.open_aan_dialog).grid(row=5, column=2, sticky='ew', pady=2)
            ttk.Button(left, text="Auto Move",  command=lambda: self.send_cmd("POSSPD 90.00 18.00 1.00")).grid(row=6, column=2, sticky='ew', pady=2)
            ttk.Button(left, text="Resist",     command=lambda: self.send_cmd(f"TORQUE {self.params['torque']:.2f}" )).grid(row=7, column=2, sticky='ew', pady=2)

            # --- NEW: Simulate Move button ---
            ttk.Button(left, text="Simulate Move", command=self.open_sim_dialog).grid(row=8, column=2, sticky='ew', pady=2)

            # Right: log
            self.log = tk.Text(right, width=74, height=12, state='disabled')
            self.log.pack(side='top', fill='both', expand=True)

            # --- Graphical joint displays ---
            gfx = ttk.LabelFrame(right, text="Visual Angles")
            gfx.pack(side='top', fill='x', pady=(6,0))
            self.elbow_view = ElbowArmCanvas(gfx, width=320, height=220)
            self.elbow_view.grid(row=0, column=0, padx=8, pady=6)
            self.wrist_view = WristDialCanvas(gfx, width=260, height=220)
            self.wrist_view.grid(row=0, column=1, padx=8, pady=6)

            # Status panel with two columns (Motor 1 & Motor 2)
            status = ttk.LabelFrame(right, text="Status (from Serial)")
            status.pack(side='bottom', fill='x', pady=(8,0))
            ttk.Label(status, text="").grid(row=0, column=0, padx=6)
            ttk.Label(status, text="Motor 1 (Elbow)", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=1, padx=6)
            ttk.Label(status, text="Motor 2 (Wrist)", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=2, padx=6)

            self.pos1_var = tk.StringVar(value="—")
            self.spd1_var = tk.StringVar(value="—")
            self.trq1_var = tk.StringVar(value="—")
            self.tmp1_var = tk.StringVar(value="—")
            self.err1_var = tk.StringVar(value="—")

            self.pos2_var = tk.StringVar(value="—")
            self.spd2_var = tk.StringVar(value="—")
            self.trq2_var = tk.StringVar(value="—")
            self.tmp2_var = tk.StringVar(value="—")
            self.err2_var = tk.StringVar(value="—")

            row = 1
            ttk.Label(status, text="Position (°):").grid(row=row, column=0, sticky="e", padx=6); ttk.Label(status, textvariable=self.pos1_var).grid(row=row, column=1, sticky="w"); ttk.Label(status, textvariable=self.pos2_var).grid(row=row, column=2, sticky="w"); row+=1
            ttk.Label(status, text="Speed (ERPM):").grid(row=row, column=0, sticky="e", padx=6); ttk.Label(status, textvariable=self.spd1_var).grid(row=row, column=1, sticky="w"); ttk.Label(status, textvariable=self.spd2_var).grid(row=row, column=2, sticky="w"); row+=1
            ttk.Label(status, text="Torque (Nm):").grid(row=row, column=0, sticky="e", padx=6); ttk.Label(status, textvariable=self.trq1_var).grid(row=row, column=1, sticky="w"); ttk.Label(status, textvariable=self.trq2_var).grid(row=row, column=2, sticky="w"); row+=1
            ttk.Label(status, text="Temp (°C):").grid(row=row, column=0, sticky="e", padx=6); ttk.Label(status, textvariable=self.tmp1_var).grid(row=row, column=1, sticky="w"); ttk.Label(status, textvariable=self.tmp2_var).grid(row=row, column=2, sticky="w"); row+=1
            ttk.Label(status, text="Error:").grid(row=row, column=0, sticky="e", padx=6, pady=(0,6)); ttk.Label(status, textvariable=self.err1_var).grid(row=row, column=1, sticky="w", pady=(0,6)); ttk.Label(status, textvariable=self.err2_var).grid(row=row, column=2, sticky="w", pady=(0,6))

            # Threads
            self._reader_stop = False
            threading.Thread(target=self.reader, daemon=True).start()
            if STATUS_POLL_ENABLED:
                threading.Thread(target=self.status_poller, daemon=True).start()

        # ---------- UI dialogs ----------
        def open_settings(self):
            dlg = tk.Toplevel(self)
            dlg.title("Settings")
            fields = [
                ("RPM", 'rpm'), ("Pos (°)", 'pos'), ("Torque (Nm)", 'torque'),
                ("Duty", 'duty'), ("Current (A)", 'current'), ("Brake (A)", 'brake'),
                ("PosSpd Pos (°)", 'posspd_p'), ("PosSpd Vel (ERPM)", 'posspd_v'), ("PosSpd Acc", 'posspd_a'),
                ("AAN Start (°)","aan_s"), ("AAN End (°)","aan_e"), ("AAN Dur (s)","aan_d"),
            ]
            self.vars = {}
            for i,(lbl,key) in enumerate(fields):
                ttk.Label(dlg, text=lbl).grid(row=i, column=0, sticky='e', padx=5, pady=2)
                v = tk.DoubleVar(value=self.params[key])
                ttk.Entry(dlg, textvariable=v, width=12).grid(row=i, column=1, pady=2)
                self.vars[key] = v
            fb = ttk.Frame(dlg)
            fb.grid(row=len(fields), column=0, columnspan=2, pady=10)
            ttk.Button(fb, text='OK', command=lambda:self._save(dlg)).grid(row=0, column=0, padx=5)
            ttk.Button(fb, text='Cancel', command=dlg.destroy).grid(row=0, column=1)

        def open_aan_dialog(self):
            dlg = tk.Toplevel(self)
            dlg.title("Assist-As-Needed Parameters")
            labels = ["Start (°):", "End (°):", "Duration (s):"]
            keys   = ['aan_s', 'aan_e', 'aan_d']
            vars_loc = {}
            for i,(lbl, key) in enumerate(zip(labels, keys)):
                ttk.Label(dlg, text=lbl).grid(row=i, column=0, sticky='e', padx=5, pady=2)
                v = tk.DoubleVar(value=self.params[key])
                ttk.Entry(dlg, textvariable=v, width=10).grid(row=i, column=1, pady=2)
                vars_loc[key] = v
            fb = ttk.Frame(dlg)
            fb.grid(row=len(labels), column=0, columnspan=2, pady=10)
            def on_ok():
                s = vars_loc['aan_s'].get()
                e = vars_loc['aan_e'].get()
                d = vars_loc['aan_d'].get()
                self.params['aan_s'], self.params['aan_e'], self.params['aan_d'] = s, e, d
                self.send_cmd(f"AAN {s:.2f} {e:.2f} {d:.2f}")
                dlg.destroy()
            ttk.Button(fb, text='OK', command=on_ok).grid(row=0, column=0, padx=5)
            ttk.Button(fb, text='Cancel', command=dlg.destroy).grid(row=0, column=1)

        def _save(self, dlg):
            for k, v in self.vars.items(): self.params[k] = v.get()
            dlg.destroy()

        # ---------- Simulate Move dialog ----------
        def open_sim_dialog(self):
            dlg = tk.Toplevel(self)
            dlg.title("Simulate Move (10 rpm)")
            dlg.resizable(False, False)

            # Current angles as defaults
            cur_elbow = getattr(self.elbow_view, "angle_deg", 0.0)
            cur_wrist = getattr(self.wrist_view, "angle_deg", 0.0)

            tk.Label(dlg, text="Elbow target (°):").grid(row=0, column=0, sticky="e", padx=6, pady=4)
            tk.Label(dlg, text="Wrist target (°):").grid(row=1, column=0, sticky="e", padx=6, pady=4)

            elbow_var = tk.DoubleVar(value=float(cur_elbow))
            wrist_var = tk.DoubleVar(value=float(cur_wrist))
            tk.Entry(dlg, textvariable=elbow_var, width=10).grid(row=0, column=1, padx=6, pady=4)
            tk.Entry(dlg, textvariable=wrist_var, width=10).grid(row=1, column=1, padx=6, pady=4)

            info = ttk.Label(dlg, text="Moves graphics only at 10 rpm (60°/s). Wrist clamped to [-90, +90].")
            info.grid(row=2, column=0, columnspan=2, padx=6, pady=(2,8))

            btns = ttk.Frame(dlg)
            btns.grid(row=3, column=0, columnspan=2, pady=6)
            def start_sim():
                tgt_elbow = float(elbow_var.get())
                tgt_wrist = max(-90.0, min(90.0, float(wrist_var.get())))
                self.start_sim_move(tgt_elbow, tgt_wrist)
                dlg.destroy()
            ttk.Button(btns, text="Start", command=start_sim).grid(row=0, column=0, padx=6)
            ttk.Button(btns, text="Cancel", command=dlg.destroy).grid(row=0, column=1, padx=6)

        def start_sim_move(self, tgt_elbow: float, tgt_wrist: float):
            """Start/replace a simulated move to the given target angles at SIM_SPEED_DEG_PER_S."""
            with self._sim_lock:
                # signal any running sim to stop
                if self._sim_thread and self._sim_thread.is_alive():
                    self._sim_stop.set()
                    # don't join on UI thread; new sim will run in parallel after stop is acknowledged
                self._sim_stop = threading.Event()
                self._sim_thread = threading.Thread(
                    target=self._sim_runner, args=(tgt_elbow, tgt_wrist, self._sim_stop), daemon=True
                )
                self._sim_thread.start()

        def _sim_runner(self, tgt_elbow: float, tgt_wrist: float, stop_event: threading.Event):
            """Move elbow & wrist graphics toward targets at constant angular speed."""
            # Grab current angles
            cur_elbow = float(getattr(self.elbow_view, "angle_deg", 0.0))
            cur_wrist = float(getattr(self.wrist_view, "angle_deg", 0.0))
            tgt_wrist = max(-90.0, min(90.0, float(tgt_wrist)))

            def step_toward(cur, tgt, max_step):
                delta = tgt - cur
                if abs(delta) <= max_step:
                    return tgt, True
                return cur + (max_step if delta > 0 else -max_step), False

            while not stop_event.is_set():
                # per-tick maximum angle change
                max_step = SIM_SPEED_DEG_PER_S * SIM_TICK_S
                cur_elbow, done_e = step_toward(cur_elbow, tgt_elbow, max_step)
                cur_wrist, done_w = step_toward(cur_wrist, tgt_wrist, max_step)

                # update graphics + numeric position fields
                self.after(0, self.elbow_view.set_angle_deg, cur_elbow)
                self.after(0, self.wrist_view.set_angle_deg, cur_wrist)
                self.after(0, self.pos1_var.set, f"{cur_elbow:.1f}")
                self.after(0, self.pos2_var.set, f"{cur_wrist:.1f}")

                if done_e and done_w:
                    break
                time.sleep(SIM_TICK_S)

        # ---------- Motor commands ----------
        def stop_all(self):
            self.send_cmd("RPM 0")
            self.send_cmd("POS 0.00")
            self.send_cmd("TORQUE 0.00")
            self.send_cmd("POSSPD 0.00 0.00 0.00")

        def send_cmd(self, cmd):
            self.ser.write((cmd+"\n").encode())
            self.log.configure(state='normal')
            self.log.insert('end', f'> {cmd}\n')
            self.log.see('end')
            self.log.configure(state='disabled')

        # ---------- Serial read + status parse ----------
        def reader(self):
            """Continuously read serial; print lines to the log and parse status if a line matches."""
            buf = b''
            while not getattr(self, "_reader_stop", False):
                try:
                    data = self.ser.read(512)
                except Exception:
                    data = b""
                if data:
                    buf += data
                    lines = buf.split(b"\n")
                    for ln in lines[:-1]:
                        txt = ln.decode(errors='ignore').strip()
                        if not txt:
                            continue
                        # 1) show in log
                        self.log.configure(state='normal')
                        self.log.insert('end', txt+'\n')
                        self.log.see('end')
                        self.log.configure(state='disabled')
                        # 2) parse status
                        self.try_parse_status(txt)
                    buf = lines[-1]
                time.sleep(0.005)

        def status_poller(self):
            """Only used if STATUS_POLL_ENABLED=True (not needed for streaming firmwares)."""
            while True:
                try:
                    self.ser.write(b"STATUS?\n")
                except Exception:
                    pass
                time.sleep(STATUS_POLL_PERIOD_S)

        # ---------- Status line parsing ----------
        def try_parse_status(self, line: str):
            """
            Accepts flexible formats, e.g.:
              elbow=45 wrist=30 erpm1=900 erpm2=120 iq1=1.5 iq2=0.8 temp1=33 temp2=31 err1=0 err2=0
              pos1=45 pos2=30 spd1=900 spd2=120 cur1=1.5 cur2=0.8
              deg=45 sup=30 spd=900 iq=1.5
            Unsuffixed fields (pos/spd/iq/torque/temp/err) map to Motor 1.
            """
            lo = line.lower()

            # Fast reject
            if not any(k in lo for k in ["pos","pos1","pos2","elbow","wrist","sup","deg","deg1","deg2",
                                         "spd","spd1","spd2","erpm","erpm1","erpm2",
                                         "iq","iq1","iq2","cur","cur1","cur2","current",
                                         "torq","torque","tq","temp","temp1","temp2","err","err1","err2"]):
                return

            def pick(keys, default=None, cast=float):
                for k in keys:
                    m = re.search(rf'\b{k}\s*[:= ]\s*(-?\d+(?:\.\d+)?)', lo)
                    if m:
                        try: return cast(m.group(1))
                        except: pass
                return default

            # Angles (deg)
            elbow_deg = pick(["elbow","pos1","p1","deg1","deg"])           # Motor 1
            wrist_deg = pick(["wrist","sup","pos2","p2","deg2"])           # Motor 2

            # Position aliases (still map to M1/M2)
            pos1 = pick(["pos1","p1"])
            pos2 = pick(["pos2","p2"])
            if elbow_deg is None: elbow_deg = pos1
            if wrist_deg is None: wrist_deg = pos2

            # Speeds (ERPM)
            spd1 = pick(["spd1","erpm1","v1","speed1","spd","erpm","speed"])
            spd2 = pick(["spd2","erpm2","v2","speed2"])

            # Currents / torques / temps / errors
            iq1  = pick(["iq1","cur1","current1","i1","iq","cur","current"])
            iq2  = pick(["iq2","cur2","current2","i2"])
            t1   = pick(["torque1","torq1","tq1","t1","torque"])
            t2   = pick(["torque2","torq2","tq2","t2"])
            tmp1 = pick(["temp1","t1c","temperature1","temp"])
            tmp2 = pick(["temp2","t2c","temperature2"])
            err1 = pick(["err1","error1","e1","err","error"], cast=int)
            err2 = pick(["err2","error2","e2"], cast=int)

            # Compute torque if needed
            if t1 is None and iq1 is not None: t1 = float(iq1) * KT_NM_PER_A_1
            if t2 is None and iq2 is not None: t2 = float(iq2) * KT_NM_PER_A_2

            # Update graphics & status
            if elbow_deg is not None:
                self.after(0, self.elbow_view.set_angle_deg, float(elbow_deg))
                self.after(0, self.pos1_var.set, f"{float(elbow_deg):.1f}")
            if wrist_deg is not None:
                wd = max(-90.0, min(90.0, float(wrist_deg)))  # clamp to dial range
                self.after(0, self.wrist_view.set_angle_deg, wd)
                self.after(0, self.pos2_var.set, f"{wd:.1f}")

            if spd1 is not None: self.after(0, self.spd1_var.set, f"{float(spd1):.0f}")
            if spd2 is not None: self.after(0, self.spd2_var.set, f"{float(spd2):.0f}")
            if t1   is not None: self.after(0, self.trq1_var.set, f"{float(t1):.2f}")
            if t2   is not None: self.after(0, self.trq2_var.set, f"{float(t2):.2f}")
            if tmp1 is not None: self.after(0, self.tmp1_var.set, f"{float(tmp1):.0f}")
            if tmp2 is not None: self.after(0, self.tmp2_var.set, f"{float(tmp2):.0f}")
            if err1 is not None: self.after(0, self.err1_var.set, f"{int(err1)}")
            if err2 is not None: self.after(0, self.err2_var.set, f"{int(err2)}")

        # ---------- Motor commands ----------
        def stop_all(self):
            self.send_cmd("RPM 0")
            self.send_cmd("POS 0.00")
            self.send_cmd("TORQUE 0.00")
            self.send_cmd("POSSPD 0.00 0.00 0.00")

        def send_cmd(self, cmd):
            self.ser.write((cmd+"\n").encode())
            self.log.configure(state='normal')
            self.log.insert('end', f'> {cmd}\n')
            self.log.see('end')
            self.log.configure(state='disabled')

        # ---------- Serial read ----------
        def reader(self):
            """Continuously read serial; print lines to the log and parse status if a line matches."""
            buf = b''
            while not getattr(self, "_reader_stop", False):
                try:
                    data = self.ser.read(512)
                except Exception:
                    data = b""
                if data:
                    buf += data
                    lines = buf.split(b"\n")
                    for ln in lines[:-1]:
                        txt = ln.decode(errors='ignore').strip()
                        if not txt:
                            continue
                        # 1) show in log
                        self.log.configure(state='normal')
                        self.log.insert('end', txt+'\n')
                        self.log.see('end')
                        self.log.configure(state='disabled')
                        # 2) parse status
                        self.try_parse_status(txt)
                    buf = lines[-1]
                time.sleep(0.005)

        def status_poller(self):
            """Only used if STATUS_POLL_ENABLED=True (not needed for streaming firmwares)."""
            while True:
                try:
                    self.ser.write(b"STATUS?\n")
                except Exception:
                    pass
                time.sleep(STATUS_POLL_PERIOD_S)

    # ---------------- Graphics widgets ----------------
    class ElbowArmCanvas(tk.Canvas):
        """
        Elbow: upper arm fixed to the LEFT from the joint; forearm rotates.
        0° = full extension to the RIGHT (+X); +deg = CCW (toward up).
        Start state: upper arm left, forearm right (180° apart), centered.
        """
        def __init__(self, master, width=320, height=220, **kw):
            super().__init__(master, width=width, height=height, bg="white", highlightthickness=1, **kw)
            # Center pivot
            self.cx, self.cy = width // 2, height // 2 + 20
            self.L1, self.L2 = ELBOW_L_UPPER, ELBOW_L_FOREARM
            self.angle_deg = 0.0
            self.create_text(width // 2, 18, text="Elbow Flexion", font=("TkDefaultFont", 10, "bold"))
            self.angle_text = self.create_text(width // 2, height - 16, text="0.0°", font=("TkDefaultFont", 10))
            # Static upper arm to LEFT; dynamic forearm to RIGHT at 0°
            self.upper_line   = self.create_line(self.cx, self.cy, self.cx - self.L1, self.cy, width=6)
            self.forearm_line = self.create_line(self.cx, self.cy, self.cx + self.L2, self.cy, width=6)
            # Elbow joint
            self.joint = self.create_oval(self.cx - 6, self.cy - 6, self.cx + 6, self.cy + 6, fill="black")
            self._redraw()

        def set_angle_deg(self, deg: float):
            self.angle_deg = float(deg)
            self._redraw()

        def _redraw(self):
            x0, y0 = self.cx, self.cy
            # 0° points to +X (right). +deg = CCW.
            a = math.radians(self.angle_deg)
            x2 = x0 + self.L2 * math.cos(a)
            y2 = y0 - self.L2 * math.sin(a)
            self.coords(self.forearm_line, x0, y0, x2, y2)
            self.itemconfigure(self.angle_text, text=f"{self.angle_deg:.1f}°")

    class WristDialCanvas(tk.Canvas):
        """
        Wrist dial: −90° (left) … 0° (right) … +90° (right).
        Numeric 0° points RIGHT; drawing rotated by +90° to align visuals with convention.
        """
        def __init__(self, master, width=260, height=220, **kw):
            super().__init__(master, width=width, height=height, bg="white", highlightthickness=1, **kw)
            self.cx, self.cy = width//2, height//2 + 10
            self.R = WRIST_DIAL_R
            self.angle_deg = 0.0
            self.create_text(width//2, 18, text="Wrist Supination", font=("TkDefaultFont", 10, "bold"))
            # Dial circle
            self.create_oval(self.cx-self.R, self.cy-self.R, self.cx+self.R, self.cy+self.R, width=2)
            # Ticks at −90..+90, drawn with +offset so numeric 0° looks RIGHT
            for t in range(-90, 91, 30):
                rt = math.radians(t + WRIST_DIAL_OFFSET_DEG)
                x1 = self.cx + (self.R-10) * math.cos(rt)
                y1 = self.cy - (self.R-10) * math.sin(rt)
                x2 = self.cx + self.R * math.cos(rt)
                y2 = self.cy - self.R * math.sin(rt)
                self.create_line(x1, y1, x2, y2, width=2)
                self.create_text(self.cx + (self.R-24) * math.cos(rt),
                                 self.cy - (self.R-24) * math.sin(rt),
                                 text=str(t), font=("TkDefaultFont", 8))
            # Pointer starting at 0° (right)
            self.pointer = self.create_line(self.cx, self.cy, self.cx + self.R - 14, self.cy, width=4)
            self.angle_text = self.create_text(width//2, height-16, text="0.0°", font=("TkDefaultFont", 10))
            self._redraw()

        def set_angle_deg(self, deg: float):
            # Clamp to valid display range
            self.angle_deg = max(-90.0, min(90.0, float(deg)))
            self._redraw()

        def _redraw(self):
            # Apply offset so numeric 0° draws to the RIGHT visually
            a = math.radians(self.angle_deg + WRIST_DIAL_OFFSET_DEG)
            x2 = self.cx + (self.R-14) * math.cos(a)
            y2 = self.cy - (self.R-14) * math.sin(a)
            self.coords(self.pointer, self.cx, self.cy, x2, y2)
            self.itemconfigure(self.angle_text, text=f"{self.angle_deg:.1f}°")

    if __name__=='__main__':
        CanGui().mainloop()
else:
    print("tkinter not available; GUI disabled.")
