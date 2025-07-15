# motor_gui.py
# GUI for sending micro-friendly serial commands: RPM, POS, TORQUE, AAN, POSSPD, and STOP behavior

import threading
import time
import serial
from serial.tools import list_ports

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    REAL_TKINTER = True
except ImportError:
    REAL_TKINTER = False

# Configuration
SERIAL_PORT = "COM8"
BAUDRATE    = 115200

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

            # Serial initialization
            ports = [p.device for p in list_ports.comports()]
            for cand in (SERIAL_PORT, f"\\.\\{SERIAL_PORT}"):
                if cand in ports:
                    try:
                        self.ser = serial.Serial(cand, BAUDRATE, timeout=0.1)
                        break
                    except:
                        pass
            else:
                messagebox.showwarning("Serial Port", f"Couldn’t open {SERIAL_PORT}; using dummy.")
                self.ser = DummySerial()

            # Parameters
            self.params = dict(
                rpm=0.0, pos=0.0, torque=0.0,
                duty=0.0, current=0.0, brake=0.0,
                posspd_p=0.0, posspd_v=0.0, posspd_a=0.0,
                aan_s=0.0, aan_e=0.0, aan_d=5.0
            )

            # Main frame layout
            frm = ttk.Frame(self, padding=10)
            frm.pack(side="left", fill="y")
            frm.grid_columnconfigure(0, weight=1)
            frm.grid_columnconfigure(1, minsize=10)
            frm.grid_columnconfigure(2, weight=1)

            # Top controls spanning all columns
            ttk.Button(frm, text="⚙", width=3, command=self.open_settings).grid(row=0, column=0, columnspan=3, pady=(0,5))
            ttk.Button(frm, text="Set Origin", style="Blue.TButton", command=lambda: self.send_cmd("ORIGIN")).grid(row=1, column=0, columnspan=3, sticky="ew", pady=2)
            ttk.Button(frm, text="STOP", style="Red.TButton", command=self.stop_all).grid(row=2, column=0, columnspan=3, sticky="ew", pady=2)

            # Column headers & separator
            ttk.Label(frm, text="TEST MODES", font=(None,10,'bold')).grid(row=3, column=0, pady=(10,2))
            ttk.Separator(frm, orient='vertical').grid(row=3, column=1, rowspan=8, sticky='ns')
            ttk.Label(frm, text="WORK MODES", font=(None,10,'bold')).grid(row=3, column=2, pady=(10,2))

            # TEST MODE buttons in col 0
            ttk.Button(frm, text="RPM",    command=lambda: self.send_cmd(f"RPM {int(self.params['rpm'])}")).grid(row=4, column=0, sticky='ew', pady=2)
            ttk.Button(frm, text="POS",    command=lambda: self.send_cmd(f"POS {self.params['pos']:.2f}")).grid(row=5, column=0, sticky='ew', pady=2)
            ttk.Button(frm, text="TORQUE", command=lambda: self.send_cmd(f"TORQUE {self.params['torque']:.2f}")).grid(row=6, column=0, sticky='ew', pady=2)
            ttk.Button(frm, text="DUTY",   command=lambda: self.send_cmd(f"DUTY {self.params['duty']:.3f}")).grid(row=7, column=0, sticky='ew', pady=2)
            ttk.Button(frm, text="CURRENT",command=lambda: self.send_cmd(f"CURRENT {self.params['current']:.2f}")).grid(row=8, column=0, sticky='ew', pady=2)
            ttk.Button(frm, text="BRAKE",  command=lambda: self.send_cmd(f"BRAKE {self.params['brake']:.2f}")).grid(row=9, column=0, sticky='ew', pady=2)
            ttk.Button(frm, text="POSSPD", command=lambda: self.send_cmd(
                f"POSSPD {self.params['posspd_p']:.2f} {self.params['posspd_v']:.2f} {self.params['posspd_a']:.2f}" )
            ).grid(row=10, column=0, sticky='ew', pady=2)

            # WORK MODE buttons in col 2
            ttk.Button(frm, text="Assist A/N", command=self.open_aan_dialog).grid(row=4, column=2, sticky='ew', pady=2)
            ttk.Button(frm, text="Auto Move",  command=lambda: self.send_cmd("POSSPD 90.00 18.00 1.00")).grid(row=5, column=2, sticky='ew', pady=2)
            ttk.Button(frm, text="Resist",     command=lambda: self.send_cmd(f"TORQUE {self.params['torque']:.2f}")).grid(row=6, column=2, sticky='ew', pady=2)

            # Log area
            self.log = tk.Text(self, width=60, height=20, state='disabled')
            self.log.pack(side='right', fill='both', expand=True)

            threading.Thread(target=self.reader, daemon=True).start()

        def open_settings(self):
            dlg = tk.Toplevel(self)
            dlg.title("Settings")
            fields = [
                ("RPM", 'rpm'), ("Pos (°)", 'pos'), ("Torque (Nm)", 'torque'),
                ("Duty", 'duty'), ("Current", 'current'), ("Brake", 'brake'),
                ("PosSpd Pos", 'posspd_p'), ("PosSpd Vel", 'posspd_v'), ("PosSpd Acc", 'posspd_a'),
                ("AAN Start","aan_s"), ("AAN End","aan_e"), ("AAN Dur","aan_d"),
            ]
            self.vars = {}
            for i,(lbl,key) in enumerate(fields):
                ttk.Label(dlg, text=lbl).grid(row=i, column=0, sticky='e', padx=5, pady=2)
                v = tk.DoubleVar(value=self.params[key])
                ttk.Entry(dlg, textvariable=v, width=10).grid(row=i, column=1, pady=2)
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
            for i,(lbl,key) in enumerate(zip(labels, keys)):
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
            for k,v in self.vars.items(): self.params[k] = v.get()
            dlg.destroy()

        def stop_all(self):
            # reset all to zero
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

        def reader(self):
            buf = b''
            while True:
                data = self.ser.read(128)
                if data:
                    buf += data
                    lines = buf.split(b"\n")
                    for ln in lines[:-1]:
                        txt = ln.decode(errors='ignore')
                        self.log.configure(state='normal')
                        self.log.insert('end', txt+'\n')
                        self.log.see('end')
                        self.log.configure(state='disabled')
                    buf = lines[-1]
                time.sleep(0.05)

    if __name__=='__main__':
        CanGui().mainloop()
else:
    print("tkinter not available; GUI disabled.")
