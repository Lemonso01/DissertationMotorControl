# motor_gui_pyside6.py
# PySide6 motor GUI
#  - Serial motor control + real-time dual-motor status
#  - Modern Elbow & Wrist visualizations
#  - "Simulate Move" at 10 rpm (60 deg/s) affecting graphics only

import sys
import threading
import time
import re
import math
import serial
from serial.tools import list_ports
import os
from datetime import datetime


from PySide6 import QtCore, QtGui, QtWidgets

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
KT_NM_PER_A_1          = 0.123   # Motor 1 (Elbow)
KT_NM_PER_A_2          = 0.078   # Motor 2 (Wrist)

GEARBOX_REDUCTION_1    = 10 # Motor 1 (Elbow)
GEARBOX_REDUCTION_2    = 6 # Motor 2 (Wrist)

# Drawing sizes
ELBOW_L_UPPER          = 110  # px
ELBOW_L_FOREARM        = 110  # px
WRIST_DIAL_R           = 70   # px
WRIST_DIAL_OFFSET_DEG  = 90   # rotate drawing so numeric 0° points RIGHT visually

#AUTOMOVE Tolerance
AUTO_TOL               = 0.5 # 0.5° of tolerance for the final position


class DummySerial:
    def __init__(self, *args, **kwargs):
        self._in = b""

    def write(self, data):
        # Loop back exactly what was sent (no extra packaging)
        self._in += data

    def read(self, n=1):
        if not self._in:
            time.sleep(0.05)
            return b""
        out, self._in = self._in, b""
        return out


# ---------------- Graphics widgets (Qt) ----------------
class ElbowArmWidget(QtWidgets.QWidget):
    """
    Elbow: upper arm fixed to the LEFT from the joint; forearm rotates.
    0° = full extension to the RIGHT (+X); +deg = CCW (toward up).
    Start state: upper arm left, forearm right (180° apart), centered.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 220)
        self.angle_deg = 0.0
        self.cx = self.width() // 2
        self.cy = self.height() // 2 + 20
        self.L1 = ELBOW_L_UPPER
        self.L2 = ELBOW_L_FOREARM

        # Colors
        self._bg_color      = QtGui.QColor("#f4f5fb")
        self._card_color    = QtGui.QColor("#ffffff")
        self._grid_color    = QtGui.QColor("#e0e3f0")
        self._text_color    = QtGui.QColor("#222222")
        self._muted_text    = QtGui.QColor("#666a7a")
        self._arm_color     = QtGui.QColor("#b0b4c5")
        self._forearm_color = QtGui.QColor("#2979ff")
        self._joint_color   = QtGui.QColor("#2979ff")

    def set_angle_deg(self, deg: float):
        self.angle_deg = float(deg)
        self.update()

    def resizeEvent(self, event):
        self.cx = self.width() // 2
        self.cy = self.height() // 2 + 20
        super().resizeEvent(event)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        rect = self.rect()

        # --- Background ---
        painter.fillRect(rect, self._bg_color)

        # --- Card panel ---
        margin = 8
        card_rect = rect.adjusted(margin, margin, -margin, -margin)
        path = QtGui.QPainterPath()
        path.addRoundedRect(card_rect, 12, 12)
        painter.fillPath(path, self._card_color)

        # Soft border
        border_pen = QtGui.QPen(QtGui.QColor(0, 0, 0, 25), 1)
        painter.setPen(border_pen)
        painter.drawPath(path)

        # --- Title ---
        painter.setPen(self._text_color)
        title_font = painter.font()
        title_font.setBold(True)
        title_font.setPointSize(10)
        painter.setFont(title_font)
        painter.drawText(
            card_rect.adjusted(0, 4, 0, 0),
            QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop,
            "Elbow Flexion / Extention"
        )

        # --- Subtitle (angle text) ---
        painter.setPen(self._muted_text)
        sub_font = painter.font()
        sub_font.setBold(False)
        sub_font.setPointSize(9)
        painter.setFont(sub_font)
        painter.drawText(
            card_rect.adjusted(10, 22, -10, 0),
            QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop,
            f"{self.angle_deg:+.1f}°"
        )

        # --- Drawing area ---
        draw_rect = card_rect.adjusted(10, 40, -10, -10)

        # Shift joint further down for more wiggle room
        x0 = draw_rect.center().x()
        y0 = draw_rect.center().y() + 20

        radius = min(draw_rect.width(), draw_rect.height()) // 2
        forearm_len = min(self.L2, radius - 6)

        # --- Grid / reference lines ---
        painter.setPen(QtGui.QPen(self._grid_color, 1))
        painter.drawLine(draw_rect.left(), y0, draw_rect.right(), y0)
        painter.drawLine(x0, draw_rect.top(), x0, draw_rect.bottom())

        # --- Range indicators (±90°) ---
        range_pen = QtGui.QPen(QtGui.QColor("#cfd2e5"), 3)
        painter.setPen(range_pen)
        for angle_deg in (-90, 0, 90):
            a = math.radians(angle_deg)
            x2 = x0 + radius * math.cos(a)
            y2 = y0 - radius * math.sin(a)
            painter.drawPoint(int(x2), int(y2))

        # --- Upper arm (static) ---
        painter.setPen(QtGui.QPen(self._arm_color, 10, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        painter.drawLine(x0, y0, x0 - self.L1, y0)

        # --- Forearm (active) ---
        a = math.radians(self.angle_deg)
        x2 = x0 + forearm_len * math.cos(a)
        y2 = y0 - forearm_len * math.sin(a)
        painter.setPen(QtGui.QPen(self._forearm_color, 10, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        painter.drawLine(x0, y0, int(x2), int(y2))

        # --- Joint circle with halo ---
        halo_radius = 11
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(self._joint_color.red(), self._joint_color.green(),
                                      self._joint_color.blue(), 60))
        painter.drawEllipse(QtCore.QPointF(x0, y0), halo_radius, halo_radius)

        painter.setBrush(self._joint_color)
        painter.drawEllipse(QtCore.QPointF(x0, y0), 6, 6)


class WristDialWidget(QtWidgets.QWidget):
    """
    Wrist dial: −90° (left) … 0° (right) … +90° (right).
    Numeric 0° points RIGHT; drawing rotated by +90° to align visuals with convention.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 220)
        self.angle_deg = 0.0
        self.cx = self.width() // 2
        self.cy = self.height() // 2 + 10
        self.R = WRIST_DIAL_R

        # Colors
        self._bg_color     = QtGui.QColor("#f4f5fb")
        self._card_color   = QtGui.QColor("#ffffff")
        self._rim_color    = QtGui.QColor("#cfd2e5")
        self._text_color   = QtGui.QColor("#222222")
        self._muted_text   = QtGui.QColor("#666a7a")
        self._tick_color   = QtGui.QColor("#a5a9ba")
        self._needle_color = QtGui.QColor("#0077ff")

    def set_angle_deg(self, deg: float):
        self.angle_deg = max(-90.0, min(90.0, float(deg)))
        self.update()

    def resizeEvent(self, event):
        self.cx = self.width() // 2
        self.cy = self.height() // 2 + 10
        super().resizeEvent(event)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        rect = self.rect()

        # --- Background ---
        painter.fillRect(rect, self._bg_color)

        # --- Card panel ---
        margin = 8
        card_rect = rect.adjusted(margin, margin, -margin, -margin)
        path = QtGui.QPainterPath()
        path.addRoundedRect(card_rect, 12, 12)
        painter.fillPath(path, self._card_color)

        border_pen = QtGui.QPen(QtGui.QColor(0, 0, 0, 25), 1)
        painter.setPen(border_pen)
        painter.drawPath(path)

        # --- Title & angle text ---
        painter.setPen(self._text_color)
        title_font = painter.font()
        title_font.setBold(True)
        title_font.setPointSize(10)
        painter.setFont(title_font)
        painter.drawText(
            card_rect.adjusted(0, 4, 0, 0),
            QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop,
            "Wrist Supination / Pronation"
        )

        painter.setPen(self._muted_text)
        sub_font = painter.font()
        sub_font.setBold(False)
        sub_font.setPointSize(9)
        painter.setFont(sub_font)
        painter.drawText(
            card_rect.adjusted(10, 22, -10, 0),
            QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop,
            f"{self.angle_deg:+.1f}°"
        )

        # --- Dial area ---
        dial_rect = card_rect.adjusted(14, 44, -14, -14)
        self.cx = dial_rect.center().x()
        self.cy = dial_rect.center().y()
        radius = min(dial_rect.width(), dial_rect.height()) // 2
        self.R = radius

        # Outer circle
        painter.setPen(QtGui.QPen(self._rim_color, 2))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawEllipse(QtCore.QPointF(self.cx, self.cy), radius, radius)

        # Major ticks & labels (−90..+90)
        painter.setPen(QtGui.QPen(self._tick_color, 2))
        tick_font = painter.font()
        tick_font.setPointSize(8)
        painter.setFont(tick_font)

        for t in range(-90, 91, 30):
            rt = math.radians(t + WRIST_DIAL_OFFSET_DEG)
            x1 = self.cx + (radius - 10) * math.cos(rt)
            y1 = self.cy - (radius - 10) * math.sin(rt)
            x2 = self.cx + radius * math.cos(rt)
            y2 = self.cy - radius * math.sin(rt)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

            lx = self.cx + (radius - 24) * math.cos(rt)
            ly = self.cy - (radius - 24) * math.sin(rt)
            painter.drawText(
                int(lx) - 12, int(ly) - 8, 24, 16,
                QtCore.Qt.AlignCenter,
                str(t)
            )

        # Minor ticks (every 10°)
        painter.setPen(QtGui.QPen(self._tick_color.lighter(130), 1))
        for t in range(-90, 91, 10):
            if t % 30 == 0:
                continue
            rt = math.radians(t + WRIST_DIAL_OFFSET_DEG)
            x1 = self.cx + (radius - 6) * math.cos(rt)
            y1 = self.cy - (radius - 6) * math.sin(rt)
            x2 = self.cx + radius * math.cos(rt)
            y2 = self.cy - radius * math.sin(rt)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        # Needle
        painter.setPen(QtGui.QPen(self._needle_color, 5, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        a = math.radians(self.angle_deg + WRIST_DIAL_OFFSET_DEG)
        nx2 = self.cx + (radius - 14) * math.cos(a)
        ny2 = self.cy - (radius - 14) * math.sin(a)
        painter.drawLine(int(self.cx), int(self.cy), int(nx2), int(ny2))

        # Center cap
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(self._needle_color))
        painter.drawEllipse(QtCore.QPointF(self.cx, self.cy), 6, 6)


# ---------------- Main Window ----------------
class MainWindow(QtWidgets.QMainWindow):
    log_signal = QtCore.Signal(str)
    status_signal = QtCore.Signal(object)

    elbow_angle_signal = QtCore.Signal(float)
    wrist_angle_signal = QtCore.Signal(float)

    pos1_text_signal = QtCore.Signal(str)
    pos2_text_signal = QtCore.Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Control GUI")

        # Serial initialization
        self.ser = None
        self.using_dummy = False
        self._init_serial()

        # Parameters (per-motor)
        base_params_1 = dict(
            rpm=0.0, pos=0.0, torque=0.0,
            duty=0.0, current=0.0, brake=0.0,
            posspd_ps=0.0, posspd_pe=0.0, posspd_v=0.0, posspd_a=1500, auto_rep=0,
            aan_s=0.0, aan_e=0.0, aan_d=5.0,
            res=0.0
        )

        base_params_2 = dict(
            rpm=0.0, pos=0.0, torque=0.0,
            duty=0.0, current=0.0, brake=0.0,
            posspd_ps=0.0, posspd_pe=0.0, posspd_v=0.0, posspd_a=600, auto_rep=0,
            aan_s=0.0, aan_e=0.0, aan_d=5.0,
            res=0.0
        )
        self.params_m = {
            1: dict(base_params_1),
            2: dict(base_params_2),
        }

        # AUTOMOVE Repetition
        self._last_pos_deg = {1: None, 2: None}  # latest telemetry position per motor

        self._auto = {
            1: {"active": False, "moves_left": 0, "target": None},
            2: {"active": False, "moves_left": 0, "target": None},
        }


        # Simulation handles
        self._sim_stop = threading.Event()
        self._sim_thread = None
        self._sim_lock = threading.Lock()

        # Reader threads control
        self._reader_stop = False

        self._build_ui()

        # ---- File logging (TX/RX) ----
        self._log_fp = None
        self._log_path = None
        self._open_log_file()


        # Connect signals
        self.log_signal.connect(self.append_log)
        self.status_signal.connect(self.apply_status_update)
        self.elbow_angle_signal.connect(self.elbow_view.set_angle_deg)
        self.wrist_angle_signal.connect(self.wrist_view.set_angle_deg)
        self.pos1_text_signal.connect(self.pos1_label.setText)
        self.pos2_text_signal.connect(self.pos2_label.setText)

        # Start reader thread
        threading.Thread(target=self.reader, daemon=True).start()
        if STATUS_POLL_ENABLED:
            threading.Thread(target=self.status_poller, daemon=True).start()


    def kt(self, motor_id: int) -> float:
        mid = 1 if int(motor_id) == 1 else 2
        return KT_NM_PER_A_1 if mid == 1 else KT_NM_PER_A_2
    
    def Ng(self, motor_id: int) -> float:
        mid = 1 if int(motor_id) == 1 else 2
        return GEARBOX_REDUCTION_1 if mid == 1 else GEARBOX_REDUCTION_2

    def resist_current_from_user_input(self, motor_id: int) -> float:
        """
        User enters Resist as torque (Nm). Convert to current (A) for BRK command:
          I = T / (Kt * N)
          
          T = I * Kt * N

          T - Torque (Nm)
          I - Current (A)
          Kt - Motor Torque constant
          N - Motor Gearbox reduction
        """
        mid = 1 if int(motor_id) == 1 else 2
        tau_nm = float(self.params_m[mid]["res"])
        kt = self.kt(mid)
        Ng = self.Ng(mid)
        if kt <= 0 or Ng <= 0:
            return 0.0
        return tau_nm / (kt * Ng)
    
    def start_auto_move(self, motor_id: int):
        mid = 1 if int(motor_id) == 1 else 2


        s = float(self.params_m[mid]["posspd_ps"])
        e = float(self.params_m[mid]["posspd_pe"])
        tol = AUTO_TOL

        try:
            rep = int(float(self.params_m[mid]["auto_rep"]))
        except Exception:
            rep = 1
        rep = max(1, rep)

        cur = self._last_pos_deg[mid]
        if cur is None:
            cur = s

        # Decide first target based on where we are
        if abs(cur - s) <= tol:
            target = e
        elif abs(cur - e) <= tol:
            target = s
        else:
            target = e  # default

        self._auto[mid]["active"] = True
        self._auto[mid]["moves_left"] = 2 * rep
        self._auto[mid]["target"] = float(target)

        # Kick off first PSA immediately
        self._send_psa(mid, target)

    def stop_auto_move(self, motor_id: int):
        mid = 1 if int(motor_id) == 1 else 2
        self._auto[mid]["active"] = False
        self._auto[mid]["moves_left"] = 0
        self._auto[mid]["target"] = None

    def _send_psa(self, mid: int, target_pos_deg: float):
        v = float(self.params_m[mid]["posspd_v"])
        a = float(self.params_m[mid]["posspd_a"])
        motor_pos = self._ui_deg_to_motor_deg(mid, target_pos_deg)
        self.send_cmd_motor(mid, f"PSA {motor_pos:.2f} {v:.2f} {a:.2f}")

    def _auto_tick_on_position(self, mid: int, pos_deg: float):
        st = self._auto[mid]
        if not st["active"] or st["moves_left"] <= 0 or st["target"] is None:
            return

        tol = AUTO_TOL
        tgt = float(st["target"])

        # Wait until we're within tolerance
        if abs(pos_deg - tgt) > tol:
            return

        # Reached current target => consume one move
        st["moves_left"] -= 1
        if st["moves_left"] <= 0:
            self.stop_auto_move(mid)
            return

        s = float(self.params_m[mid]["posspd_ps"])
        e = float(self.params_m[mid]["posspd_pe"])

        # Toggle target
        next_tgt = e if abs(tgt - s) <= tol else s
        st["target"] = float(next_tgt)

        # Schedule the next PSA after 1 second (non-blocking)
        QtCore.QTimer.singleShot(1000, lambda mid=mid, t=next_tgt: self._send_psa(mid, t))

    def _open_log_file(self):
        # Create a logs folder next to your script
        base_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(base_dir, "logs")
        os.makedirs(log_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = os.path.join(log_dir, f"serial_log_{ts}.txt")
        self._log_fp = open(self._log_path, "a", encoding="utf-8", buffering=1)  # line-buffered

        # Write header
        self._log_fp.write(f"Serial Log Started: {datetime.now().isoformat(timespec='seconds')}\n")
        self._log_fp.write(f"Port: {SERIAL_PORT}  Baud: {BAUDRATE}\n")
        self._log_fp.write("-" * 60 + "\n")

    def _write_log_line(self, line: str):
        #Write a line to file
        if not self._log_fp:
            return
        self._log_fp.write(f"{line}\n")

    def _ui_deg_to_motor_deg(self, mid: int, deg: float) -> float:
        """Convert GUI/user-facing degrees -> motor convention."""
        x = float(deg)
        return -x if mid == 1 else x   # reverse only elbow motor

    def _motor_deg_to_ui_deg(self, mid: int, deg: float) -> float:
        """Convert motor feedback degrees -> GUI/user-facing convention."""
        x = float(deg)
        return -x if mid == 1 else x   # reverse only elbow motor




    # ---------- Serial init ----------
    def _init_serial(self):
        ports = [p.device for p in list_ports.comports()]
        self.using_dummy = False
        chosen = None
        for cand in (SERIAL_PORT, f"\\\\.\\{SERIAL_PORT}"):
            if cand in ports:
                chosen = cand
                break

        if chosen is not None:
            try:
                self.ser = serial.Serial(chosen, BAUDRATE, timeout=0.05)
                return
            except Exception:
                pass

        QtWidgets.QMessageBox.warning(
            self, "Serial Port",
            f"Couldn't open {SERIAL_PORT}; using dummy serial device."
        )
        self.ser = DummySerial()
        self.using_dummy = True

    # ---------- Per-motor param getter ----------
    def p(self, motor_id: int, key: str) -> float:
        mid = 1 if int(motor_id) == 1 else 2
        return float(self.params_m[mid][key])

    # ---------- Motor-targeted command helper ----------
    def send_cmd_motor(self, motor_id: int, cmd: str):
        """
        Insert motor selection AFTER the command token:
          "TORQUE 1 0.00"
          "POS 2 10.00"
          "PSA 1 90.00 18.00 1.00"
        """
        mid = 1 if int(motor_id) == 1 else 2
        cmd = cmd.strip()
        if not cmd:
            return

        parts = cmd.split()
        head = parts[0]
        tail = parts[1:]
        rebuilt = " ".join([head, str(mid)] + tail)
        self.send_cmd(rebuilt)

    # ---------- UI ----------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # Left panel (buttons)
        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QGridLayout(left)
        left_layout.setColumnStretch(0, 1)
        left_layout.setColumnStretch(2, 1)

        # Settings / main controls
        btn_settings = QtWidgets.QPushButton("⚙")
        btn_settings.setFixedWidth(40)
        btn_settings.clicked.connect(self.open_settings)
        left_layout.addWidget(btn_settings, 0, 0, 1, 3)

        btn_origin = QtWidgets.QPushButton("SET ORIGIN")
        btn_origin.setStyleSheet("background-color: #1976d2; color: white; font-weight:bold;")
        btn_origin.clicked.connect(lambda: self.send_cmd("ORIGIN 1"))
        left_layout.addWidget(btn_origin, 1, 0, 1, 3)

        btn_calib = QtWidgets.QPushButton("CALIBRATE")
        btn_calib.setStyleSheet("background-color: #ff9800; color: white; font-weight:bold;")
        btn_calib.clicked.connect(lambda: self.send_cmd("CALIBRATE"))
        left_layout.addWidget(btn_calib, 2, 0, 1, 3)

         # --- STOP buttons ---
        # Row 3: Stop 1 + Stop 2
        stop_small_row = QtWidgets.QHBoxLayout()
        btn_stop1 = QtWidgets.QPushButton("STOP 1")
        btn_stop2 = QtWidgets.QPushButton("STOP 2")

        btn_stop1.setFixedWidth(70)
        btn_stop2.setFixedWidth(70)

        btn_stop1.setStyleSheet("background-color: #d32f2f; color: white; font-weight:bold;")
        btn_stop2.setStyleSheet("background-color: #d32f2f; color: white; font-weight:bold;")

        btn_stop1.clicked.connect(lambda: (self.stop_auto_move(1), self.send_cmd("STOP 1")))
        btn_stop2.clicked.connect(lambda: (self.stop_auto_move(2), self.send_cmd("STOP 2")))

        btn_stop1.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        btn_stop2.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        stop_small_row = QtWidgets.QHBoxLayout()
        stop_small_row.setContentsMargins(0, 0, 0, 0)
        stop_small_row.setSpacing(8)
        stop_small_row.addWidget(btn_stop1, 1)
        stop_small_row.addWidget(btn_stop2, 1)

        left_layout.addLayout(stop_small_row, 3, 0, 1, 3)

        # Row 4-5: STOP ALL
        btn_stopall = QtWidgets.QPushButton("STOP ALL")
        btn_stopall.setStyleSheet("background-color: #b71c1c; color: yellow; font-weight:bold; font-size:18px;")
        btn_stopall.clicked.connect(lambda: (self.stop_auto_move(1), self.stop_auto_move(2), self.send_cmd("STOPALL")))

        # Expand + taller feel (row-span already gives height; this helps visually)
        btn_stopall.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        btn_stopall.setMinimumHeight(90)

        # Add to grid, spanning 2 rows and all 3 columns
        left_layout.addWidget(btn_stopall, 4, 0, 2, 3)

        # --- Extras menu button ---
        extras_btn = QtWidgets.QPushButton("EXTRAS")
        extras_btn.setStyleSheet("background-color: #455a64; color: white; font-weight:bold;")
        left_layout.addWidget(extras_btn, 6, 0, 1, 3)

        extras_menu = QtWidgets.QMenu(extras_btn)

        m1_menu = extras_menu.addMenu("Motor 1 (Elbow) Extras")

        m1_menu.addSeparator()
        act_extras_settings_1 = m1_menu.addAction("Extras Settings")
        act_extras_settings_1.triggered.connect(lambda: self.open_extras_settings(1))


        m2_menu = extras_menu.addMenu("Motor 2 (Wrist) Extras")

        m2_menu.addSeparator()
        act_extras_settings_2 = m2_menu.addAction("Extras Settings")
        act_extras_settings_2.triggered.connect(lambda: self.open_extras_settings(2))

        # Show menu directly under the button
        extras_btn.clicked.connect(lambda: extras_menu.exec(extras_btn.mapToGlobal(QtCore.QPoint(0, extras_btn.height()))))

        # Headers: Motor 1 / Motor 2
        lbl_m1 = QtWidgets.QLabel("ELBOW MOTOR")
        lbl_m1.setStyleSheet("font-weight:bold;")
        left_layout.addWidget(lbl_m1, 7, 0, 1, 1)

        vline = QtWidgets.QFrame()
        vline.setFrameShape(QtWidgets.QFrame.VLine)
        vline.setFrameShadow(QtWidgets.QFrame.Sunken)
        left_layout.addWidget(vline, 7, 1, 4, 1)

        lbl_m2 = QtWidgets.QLabel("WRIST MOTOR")
        lbl_m2.setStyleSheet("font-weight:bold;")
        left_layout.addWidget(lbl_m2, 7, 2, 1, 1)

    

        # ---------------- Motor 1 buttons ----------------
        row_m1 = 8

        # Visible main controls
        btn_posspd_1 = QtWidgets.QPushButton("ACTIVE ASSIST")
        btn_posspd_1.clicked.connect(lambda: self.start_auto_move(1))
        left_layout.addWidget(btn_posspd_1, row_m1, 0); row_m1 += 1

        btn_aan_1 = QtWidgets.QPushButton("ASSIST A/N")
        btn_aan_1.clicked.connect(lambda: self.open_aan_dialog(motor_id=1))
        left_layout.addWidget(btn_aan_1, row_m1, 0); row_m1 += 1

        btn_res_1 = QtWidgets.QPushButton("ACTIVE RESIST")
        btn_res_1.clicked.connect(lambda: self.send_cmd_motor(1, f"BRK {self.resist_current_from_user_input(1):.2f}"))
        left_layout.addWidget(btn_res_1, row_m1, 0); row_m1 += 1

        # Extras (in menu)
        act_rpm_1 = m1_menu.addAction("RPM")
        act_rpm_1.triggered.connect(lambda: self.send_cmd_motor(1, f"RPM {int(self.p(1,'rpm'))}"))

        act_pos_1 = m1_menu.addAction("POS")
        act_pos_1.triggered.connect(lambda: self.send_cmd_motor(1, f"POS {self._ui_deg_to_motor_deg(1, self.p(1,'pos')):.2f}"))

        act_torque_1 = m1_menu.addAction("TORQUE")
        act_torque_1.triggered.connect(lambda: self.send_cmd_motor(1, f"TORQUE {self.p(1,'torque'):.2f}"))

        act_duty_1 = m1_menu.addAction("DUTY")
        act_duty_1.triggered.connect(lambda: self.send_cmd_motor(1, f"DUTY {self.p(1,'duty'):.3f}"))

        act_current_1 = m1_menu.addAction("CURRENT")
        act_current_1.triggered.connect(lambda: self.send_cmd_motor(1, f"CURRENT {self.p(1,'current'):.2f}"))

        act_brake_1 = m1_menu.addAction("Brake")
        act_brake_1.triggered.connect(lambda: self.send_cmd_motor(1, f"BRK {self.p(1,'brake'):.2f}"))

        act_psa_1 = m1_menu.addAction("PSA Start")
        act_psa_1.triggered.connect(lambda: self.send_cmd_motor(1, f"PSA {self._ui_deg_to_motor_deg(1, self.p(1,'posspd_ps')):.2f} "f"{self.p(1,'posspd_v'):.2f} {self.p(1,'posspd_a'):.2f}"))

        
        m1_menu.addSeparator()

        act_sim_1 = m1_menu.addAction("Simulate Move")
        act_sim_1.triggered.connect(self.open_sim_dialog)


        # ---------------- Motor 2 buttons ----------------
        row_m2 = 8

        # Visible main controls
        btn_posspd_2 = QtWidgets.QPushButton("ACTIVE ASSIST")
        btn_posspd_2.clicked.connect(lambda: self.start_auto_move(2))
        left_layout.addWidget(btn_posspd_2, row_m2, 2); row_m2 += 1

        btn_aan_2 = QtWidgets.QPushButton("ASSIST A/N")
        btn_aan_2.clicked.connect(lambda: self.open_aan_dialog(motor_id=2))
        left_layout.addWidget(btn_aan_2, row_m2, 2); row_m2 += 1

        btn_res_2 = QtWidgets.QPushButton("ACTIVE RESIST")
        btn_res_2.clicked.connect(lambda: self.send_cmd_motor(2, f"BRK {self.resist_current_from_user_input(2):.2f}"))
        left_layout.addWidget(btn_res_2, row_m2, 2); row_m2 += 1

        # Extras (in menu)
        act_rpm_2 = m2_menu.addAction("RPM")
        act_rpm_2.triggered.connect(lambda: self.send_cmd_motor(2, f"RPM {int(self.p(2,'rpm'))}"))

        act_pos_2 = m2_menu.addAction("POS")
        act_pos_2.triggered.connect(lambda: self.send_cmd_motor(2, f"POS {self.p(2,'pos'):.2f}"))

        act_torque_2 = m2_menu.addAction("TORQUE")
        act_torque_2.triggered.connect(lambda: self.send_cmd_motor(2, f"TORQUE {self.p(2,'torque'):.2f}"))

        act_duty_2 = m2_menu.addAction("DUTY")
        act_duty_2.triggered.connect(lambda: self.send_cmd_motor(2, f"DUTY {self.p(2,'duty'):.3f}"))

        act_current_2 = m2_menu.addAction("CURRENT")
        act_current_2.triggered.connect(lambda: self.send_cmd_motor(2, f"CURRENT {self.p(2,'current'):.2f}"))

        act_brake_2 = m2_menu.addAction("Brake")
        act_brake_2.triggered.connect(lambda: self.send_cmd_motor(2, f"BRK {self.p(2,'brake'):.2f}"))

        act_psa_2 = m2_menu.addAction("PSA Start")
        act_psa_2.triggered.connect(lambda: self.send_cmd_motor(2, f"PSA {self.p(2,'posspd_ps'):.2f} {self.p(2,'posspd_v'):.2f} {self.p(2,'posspd_a'):.2f}"))
        
        m2_menu.addSeparator()

        act_sim_2 = m2_menu.addAction("Simulate Move")
        act_sim_2.triggered.connect(self.open_sim_dialog)

        # ---------------- Combined Motor buttons ----------------

        extra_section_row = max(row_m1, row_m2)

        extra_header = QtWidgets.QLabel("BOTH MOTORS")
        extra_header.setStyleSheet("font-weight:bold; margin-top:6px;")
        extra_header.setAlignment(QtCore.Qt.AlignCenter)

        left_layout.addWidget(extra_header, extra_section_row, 0, 1, 3)

        def start_auto_move_both():
            self.start_auto_move(1)
            QtCore.QTimer.singleShot(20, lambda: self.start_auto_move(2))

        def resist_both():
            self.send_cmd_motor(1, f"BRK {self.resist_current_from_user_input(1):.2f}")
            QtCore.QTimer.singleShot(20, lambda: self.send_cmd_motor(2, f"BRK {self.resist_current_from_user_input(2):.2f}"))

        def aan_both():
            s1 = self._ui_deg_to_motor_deg(1, self.p(1,'aan_s'))
            e1 = self._ui_deg_to_motor_deg(1, self.p(1,'aan_e'))
            self.send_cmd_motor(1, f"AAN {s1:.2f} {e1:.2f} {self.p(1,'aan_d'):.2f}")
            QtCore.QTimer.singleShot(20, lambda: self.send_cmd_motor(2, f"AAN {self.p(2,'aan_s'):.2f} {self.p(2,'aan_e'):.2f} {self.p(2,'aan_d'):.2f}"))

        btn_pospd_both = QtWidgets.QPushButton("ACTIVE ASSIST")
        btn_pospd_both.clicked.connect(start_auto_move_both)

        btn_res_both = QtWidgets.QPushButton("ACTIVE RESIST")
        btn_res_both.clicked.connect(resist_both)

        btn_aan_both = QtWidgets.QPushButton("ASSIST A/N")
        btn_aan_both.clicked.connect(aan_both)

        for b in (btn_pospd_both, btn_res_both, btn_aan_both):
            b.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        left_layout.addWidget(btn_pospd_both, extra_section_row + 1, 0, 1, 3)
        left_layout.addWidget(btn_res_both, extra_section_row + 2, 0, 1, 3)
        left_layout.addWidget(btn_aan_both, extra_section_row + 3, 0, 1, 3)

        left_layout.setRowStretch(extra_section_row + 4, 1)


        # Right panel (log + graphics + status)
        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        right_layout.setSpacing(6)

        # Log (top)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1000)
        self.log.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        right_layout.addWidget(self.log, stretch=2)

        # Graphics group
        gfx_group = QtWidgets.QGroupBox("Visual Angles")
        gfx_layout = QtWidgets.QHBoxLayout(gfx_group)
        gfx_layout.setContentsMargins(8, 8, 8, 8)
        gfx_layout.setSpacing(16)
        self.elbow_view = ElbowArmWidget()
        self.wrist_view = WristDialWidget()
        gfx_layout.addWidget(self.elbow_view)
        gfx_layout.addWidget(self.wrist_view)
        right_layout.addWidget(gfx_group, stretch=1)

        # Status group
        status_group = QtWidgets.QGroupBox("Status (from Serial)")
        status_layout = QtWidgets.QGridLayout(status_group)

        status_layout.addWidget(QtWidgets.QLabel(""), 0, 0)
        status_layout.addWidget(QtWidgets.QLabel("Motor 1 (Elbow)"), 0, 1)
        status_layout.addWidget(QtWidgets.QLabel("Motor 2 (Wrist)"), 0, 2)

        # Create labels
        self.pos1_label = QtWidgets.QLabel("—")
        self.spd1_label = QtWidgets.QLabel("—")
        self.trq1_label = QtWidgets.QLabel("—")
        self.tmp1_label = QtWidgets.QLabel("—")
        self.err1_label = QtWidgets.QLabel("—")

        self.pos2_label = QtWidgets.QLabel("—")
        self.spd2_label = QtWidgets.QLabel("—")
        self.trq2_label = QtWidgets.QLabel("—")
        self.tmp2_label = QtWidgets.QLabel("—")
        self.err2_label = QtWidgets.QLabel("—")

        row_s = 1
        status_layout.addWidget(QtWidgets.QLabel("Position (°):"), row_s, 0, alignment=QtCore.Qt.AlignRight)
        status_layout.addWidget(self.pos1_label, row_s, 1)
        status_layout.addWidget(self.pos2_label, row_s, 2); row_s += 1

        status_layout.addWidget(QtWidgets.QLabel("Speed (RPM):"), row_s, 0, alignment=QtCore.Qt.AlignRight)
        status_layout.addWidget(self.spd1_label, row_s, 1)
        status_layout.addWidget(self.spd2_label, row_s, 2); row_s += 1

        status_layout.addWidget(QtWidgets.QLabel("Current (A):"), row_s, 0, alignment=QtCore.Qt.AlignRight)
        status_layout.addWidget(self.trq1_label, row_s, 1)
        status_layout.addWidget(self.trq2_label, row_s, 2); row_s += 1

        status_layout.addWidget(QtWidgets.QLabel("Temp (°C):"), row_s, 0, alignment=QtCore.Qt.AlignRight)
        status_layout.addWidget(self.tmp1_label, row_s, 1)
        status_layout.addWidget(self.tmp2_label, row_s, 2); row_s += 1

        status_layout.addWidget(QtWidgets.QLabel("Error:"), row_s, 0, alignment=QtCore.Qt.AlignRight)
        status_layout.addWidget(self.err1_label, row_s, 1)
        status_layout.addWidget(self.err2_label, row_s, 2)

        right_layout.addWidget(status_group, stretch=0)

        main_layout.addWidget(left, stretch=0)
        main_layout.addWidget(right, stretch=1)

    # ---------- Logging ----------
    @QtCore.Slot(str)
    def append_log(self, text: str):
        self.log.appendPlainText(text)
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())
        self._write_log_line(text)

    # ---------- Settings dialog (two columns: Motor 1 / Motor 2) ----------
    def open_settings(self):
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("Settings")
        layout = QtWidgets.QGridLayout(dlg)

        def add_hline(row: int):
            line = QtWidgets.QFrame()
            line.setFrameShape(QtWidgets.QFrame.HLine)
            line.setFrameShadow(QtWidgets.QFrame.Sunken)
            layout.addWidget(line, row, 0, 1, 3)  # span all columns
            return row + 1

        def add_section_title(row: int, title: str):
            lbl = QtWidgets.QLabel(title)
            lbl.setStyleSheet("font-weight:bold; margin-top:6px;")
            layout.addWidget(lbl, row, 0, 1, 3)  # span all columns
            return row + 1

        # Header row
        hdr0 = QtWidgets.QLabel("")
        hdr1 = QtWidgets.QLabel("Elbow Motor")
        hdr2 = QtWidgets.QLabel("Wrist Motor")
        hdr1.setStyleSheet("font-weight:bold;")
        hdr2.setStyleSheet("font-weight:bold;")
        layout.addWidget(hdr0, 0, 0)
        layout.addWidget(hdr1, 0, 1)
        layout.addWidget(hdr2, 0, 2)

        fields = [
            #("RPM", 'rpm'),
            #("Pos (°)", 'pos'),
            #("Torque (Nm)", 'torque'),
            #("Duty", 'duty'),
            #("Current (A)", 'current'),

            #ACTIVE ASSIST Mode
            ("ACTIVE ASSIST Repetitions", 'auto_rep'),
            ("ACTIVE ASSIST Start (°)", 'posspd_ps'),
            ("ACTIVE ASSIST End (°)", 'posspd_pe'),
            ("ACTIVE ASSIST (RPM)", 'posspd_v'),
            ("ACTIVE ASSIST (deg/s²)", 'posspd_a'),

            #Resist Mode
            ("ACTIVE Resist (Nm)", 'res'),

            #AAN Mode
            ("AAN Start (°)", "aan_s"),
            ("AAN End (°)", "aan_e"),
            ("AAN RPM (RPM)", "aan_d"),
        ]

        w1 = {}
        w2 = {}

        row = 1

        # --- ACTIVE ASSIST (PSA) section ---
        row = add_section_title(row, "ACTIVE ASSIST (PSA)")
        for label, key in [
            ("ACTIVE ASSIST Repetitions", 'auto_rep'),
            ("ACTIVE ASSIST Start (°)", 'posspd_ps'),
            ("ACTIVE ASSIST End (°)",   'posspd_pe'),
            ("ACTIVE ASSIST (RPM)",     'posspd_v'),
            ("ACTIVE ASSIST (deg/s²)",  'posspd_a'),
        ]:
            layout.addWidget(QtWidgets.QLabel(label + ":"), row, 0)
            e1 = QtWidgets.QLineEdit(str(self.params_m[1][key]))
            e2 = QtWidgets.QLineEdit(str(self.params_m[2][key]))
            layout.addWidget(e1, row, 1)
            layout.addWidget(e2, row, 2)
            w1[key] = e1
            w2[key] = e2
            row += 1

        row = add_hline(row)

        # --- ACTIVE RESIST section ---
        row = add_section_title(row, "ACTIVE RESIST")
        for label, key in [
            ("ACTIVE Resist (Nm)", 'res'),
        ]:
            layout.addWidget(QtWidgets.QLabel(label + ":"), row, 0)
            e1 = QtWidgets.QLineEdit(str(self.params_m[1][key]))
            e2 = QtWidgets.QLineEdit(str(self.params_m[2][key]))
            layout.addWidget(e1, row, 1)
            layout.addWidget(e2, row, 2)
            w1[key] = e1
            w2[key] = e2
            row += 1

        row = add_hline(row)

        # --- ASSIST AS NEEDED section ---
        row = add_section_title(row, "ASSIST AS NEEDED (AAN)")
        for label, key in [
            ("AAN Start (°)", "aan_s"),
            ("AAN End (°)",   "aan_e"),
            ("AAN RPM (RPM)", "aan_d"),
        ]:
            layout.addWidget(QtWidgets.QLabel(label + ":"), row, 0)
            e1 = QtWidgets.QLineEdit(str(self.params_m[1][key]))
            e2 = QtWidgets.QLineEdit(str(self.params_m[2][key]))
            layout.addWidget(e1, row, 1)
            layout.addWidget(e2, row, 2)
            w1[key] = e1
            w2[key] = e2
            row += 1


        btn_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel,
            parent=dlg
        )
        layout.addWidget(btn_box, row, 0, 1, 3)

        def on_ok():
            try:
                for key, le in w1.items():
                    self.params_m[1][key] = float(le.text())
                for key, le in w2.items():
                    self.params_m[2][key] = float(le.text())
            except ValueError:
                QtWidgets.QMessageBox.warning(dlg, "Invalid input", "Please enter numeric values.")
                return
            dlg.accept()

        btn_box.accepted.connect(on_ok)
        btn_box.rejected.connect(dlg.reject)
        dlg.exec()

    def open_extras_settings(self, motor_id: int):
        mid = 1 if int(motor_id) == 1 else 2

        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle(f"Extras Settings (Motor {mid})")
        layout = QtWidgets.QFormLayout(dlg)

        # Extras parameters you actually use in the menu actions
        le_rpm     = QtWidgets.QLineEdit(str(self.params_m[mid]["rpm"]))
        le_pos     = QtWidgets.QLineEdit(str(self.params_m[mid]["pos"]))
        le_torque  = QtWidgets.QLineEdit(str(self.params_m[mid]["torque"]))
        le_duty    = QtWidgets.QLineEdit(str(self.params_m[mid]["duty"]))
        le_current = QtWidgets.QLineEdit(str(self.params_m[mid]["current"]))
        le_res     = QtWidgets.QLineEdit(str(self.params_m[mid]["res"]))

        layout.addRow("RPM (command value):", le_rpm)
        layout.addRow("POS (°):",            le_pos)
        layout.addRow("TORQUE (Nm):",        le_torque)
        layout.addRow("DUTY:",               le_duty)
        layout.addRow("CURRENT (A):",        le_current)
        layout.addRow("Resist Torque (Nm):", le_res)

        btn_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel,
            parent=dlg
        )
        layout.addRow(btn_box)

        def on_ok():
            try:
                self.params_m[mid]["rpm"]     = float(le_rpm.text())
                self.params_m[mid]["pos"]     = float(le_pos.text())
                self.params_m[mid]["torque"]  = float(le_torque.text())
                self.params_m[mid]["duty"]    = float(le_duty.text())
                self.params_m[mid]["current"] = float(le_current.text())
                self.params_m[mid]["res"]     = float(le_res.text())
            except ValueError:
                QtWidgets.QMessageBox.warning(dlg, "Invalid input", "Please enter numeric values.")
                return
            dlg.accept()

        btn_box.accepted.connect(on_ok)
        btn_box.rejected.connect(dlg.reject)
        dlg.exec()


    # ---------- AAN dialog (motor-aware) ----------
    def open_aan_dialog(self, motor_id: int = 1):
        mid = 1 if int(motor_id) == 1 else 2

        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle(f"Assist-As-Needed Parameters (Motor {mid})")
        layout = QtWidgets.QFormLayout(dlg)

        le_start = QtWidgets.QLineEdit(str(self.params_m[mid]['aan_s']))
        le_end   = QtWidgets.QLineEdit(str(self.params_m[mid]['aan_e']))
        le_dur   = QtWidgets.QLineEdit(str(self.params_m[mid]['aan_d']))

        layout.addRow("Start (°):", le_start)
        layout.addRow("End (°):",   le_end)
        layout.addRow("Speed (RPM):", le_dur)

        btn_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel,
            parent=dlg
        )
        layout.addRow(btn_box)

        def on_ok():
            try:
                s = float(le_start.text())
                e = float(le_end.text())
                d = float(le_dur.text())
            except ValueError:
                QtWidgets.QMessageBox.warning(dlg, "Invalid input", "Please enter numeric values.")
                return

            self.params_m[mid]['aan_s'] = s
            self.params_m[mid]['aan_e'] = e
            self.params_m[mid]['aan_d'] = d

            s_m = self._ui_deg_to_motor_deg(mid, s)
            e_m = self._ui_deg_to_motor_deg(mid, e)
            self.send_cmd_motor(mid, f"AAN {s_m:.2f} {e_m:.2f} {d:.2f}")
            dlg.accept()

        btn_box.accepted.connect(on_ok)
        btn_box.rejected.connect(dlg.reject)
        dlg.exec()

    # ---------- Simulate Move dialog ----------
    def open_sim_dialog(self):
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("Simulate Move (10 rpm)")
        layout = QtWidgets.QGridLayout(dlg)

        # Current angles as defaults
        cur_elbow = float(self.elbow_view.angle_deg)
        cur_wrist = float(self.wrist_view.angle_deg)

        layout.addWidget(QtWidgets.QLabel("Elbow target (°):"), 0, 0)
        layout.addWidget(QtWidgets.QLabel("Wrist target (°):"), 1, 0)

        elbow_edit = QtWidgets.QLineEdit(str(cur_elbow))
        wrist_edit = QtWidgets.QLineEdit(str(cur_wrist))
        layout.addWidget(elbow_edit, 0, 1)
        layout.addWidget(wrist_edit, 1, 1)

        info = QtWidgets.QLabel("Moves graphics only at 10 rpm (60°/s). Wrist clamped to [-90, +90].")
        info.setWordWrap(True)
        layout.addWidget(info, 2, 0, 1, 2)

        btn_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel,
            parent=dlg
        )
        layout.addWidget(btn_box, 3, 0, 1, 2)

        def on_ok():
            try:
                tgt_elbow = float(elbow_edit.text())
                tgt_wrist = float(wrist_edit.text())
            except ValueError:
                QtWidgets.QMessageBox.warning(dlg, "Invalid input", "Please enter numeric values.")
                return
            tgt_wrist = max(-90.0, min(90.0, tgt_wrist))
            self.start_sim_move(tgt_elbow, tgt_wrist)
            dlg.accept()

        btn_box.accepted.connect(on_ok)
        btn_box.rejected.connect(dlg.reject)
        dlg.exec()

    def start_sim_move(self, tgt_elbow: float, tgt_wrist: float):
        """Start/replace a simulated move to the given target angles at SIM_SPEED_DEG_PER_S."""
        with self._sim_lock:
            if self._sim_thread and self._sim_thread.is_alive():
                self._sim_stop.set()
            self._sim_stop = threading.Event()
            self._sim_thread = threading.Thread(
                target=self._sim_runner,
                args=(tgt_elbow, tgt_wrist, self._sim_stop),
                daemon=True
            )
            self._sim_thread.start()

    def _sim_runner(self, tgt_elbow: float, tgt_wrist: float, stop_event: threading.Event):
        cur_elbow = float(self.elbow_view.angle_deg)
        cur_wrist = float(self.wrist_view.angle_deg)
        tgt_wrist = max(-90.0, min(90.0, float(tgt_wrist)))

        def step_toward(cur, tgt, max_step):
            delta = tgt - cur
            if abs(delta) <= max_step:
                return tgt, True
            return cur + (max_step if delta > 0 else -max_step), False

        while not stop_event.is_set():
            max_step = SIM_SPEED_DEG_PER_S * SIM_TICK_S
            cur_elbow, done_e = step_toward(cur_elbow, tgt_elbow, max_step)
            cur_wrist, done_w = step_toward(cur_wrist, tgt_wrist, max_step)

            self.elbow_angle_signal.emit(cur_elbow)
            self.wrist_angle_signal.emit(cur_wrist)
            self.pos1_text_signal.emit(f"{cur_elbow:.1f}")
            self.pos2_text_signal.emit(f"{cur_wrist:.1f}")

            if done_e and done_w:
                break
            time.sleep(SIM_TICK_S)

    # ---------- Motor commands ----------
    def send_cmd(self, cmd: str):
        try:
            self.ser.write((cmd + "\n").encode())
        except Exception:
            pass
        self.log_signal.emit(f"TX: {cmd}")

    def reader(self):
        buf = b""
        while not self._reader_stop:
            try:
                data = self.ser.read(512)
            except Exception:
                data = b""
            if data:
                buf += data
                lines = buf.split(b"\n")
                for ln in lines[:-1]:
                    try:
                        txt = ln.decode(errors="ignore").strip()
                        if not txt:
                            continue
                        self.log_signal.emit(f"RX: {txt}")
                        status = self.parse_status_line(txt)
                        if status:
                            self.status_signal.emit(status)
                    except Exception as e:
                        print("Reader error:", e)
                buf = lines[-1]
            time.sleep(0.005)

    def status_poller(self):
        while not self._reader_stop:
            try:
                self.ser.write(b"STATUS?\n")
            except Exception:
                pass
            time.sleep(STATUS_POLL_PERIOD_S)

    # ---------- Status line parsing ----------
    def parse_status_line(self, line: str):
        """
        Expected format (timestamped):
        <t_ms>,SERVO,<motor_id>,<pos_deg>,<spd_erpm>,<cur_a>,<temp_c>,<err>

        We ignore everything before and including "SERVO", then parse from motor_id onwards.
        Example:
        123456,SERVO,1,10.0,1200,2.50,38,0
        """
        s = line.strip()
        if not s:
            return None

        parts = [p.strip() for p in s.split(",")]

        # Find "SERVO" token anywhere in the line
        try:
            i = next(idx for idx, tok in enumerate(parts) if tok.upper() == "SERVO")
        except StopIteration:
            return None

        # Need: motor_id, pos, spd, cur, temp, err -> 6 fields after SERVO
        if len(parts) < i + 1 + 6:
            return None

        try:
            motor_id = int(parts[i + 1])
            pos_deg  = float(parts[i + 2])
            spd_erpm = float(parts[i + 3])
            cur_a    = float(parts[i + 4])
            temp_c   = float(parts[i + 5])
            err      = int(parts[i + 6])
        except ValueError:
            return None

        status = {}

        if motor_id == 1:
            pos_deg_ui = self._motor_deg_to_ui_deg(1, pos_deg)
            status["elbow_deg"] = pos_deg_ui
            status["spd1"]      = spd_erpm / (21 * 10)
            status["tmp1"]      = temp_c
            status["err1"]      = err
            status["t1"]        = cur_a 

        elif motor_id == 2:
            status["wrist_deg"] = pos_deg
            status["spd2"]      = spd_erpm / (14 * 6)
            status["tmp2"]      = temp_c
            status["err2"]      = err
            status["t2"]        = cur_a 

        else:
            return None

        return status




    @QtCore.Slot(object)
    def apply_status_update(self, status: object):
        if not isinstance(status, dict):
            return

        if "elbow_deg" in status:
            val = float(status["elbow_deg"])
            self._last_pos_deg[1] = val
            self.elbow_view.set_angle_deg(val)
            self.pos1_label.setText(f"{val:.1f}")
            self._auto_tick_on_position(1, val)

        if "wrist_deg" in status:
            val = float(status["wrist_deg"])
            self._last_pos_deg[2] = val
            self.wrist_view.set_angle_deg(val)
            self.pos2_label.setText(f"{val:.1f}")
            self._auto_tick_on_position(2, val)


        if "spd1" in status:
            self.spd1_label.setText(f"{float(status['spd1']):.0f}")
        if "spd2" in status:
            self.spd2_label.setText(f"{float(status['spd2']):.0f}")

        if "t1" in status:
            self.trq1_label.setText(f"{float(status['t1']):.2f}")
        if "t2" in status:
            self.trq2_label.setText(f"{float(status['t2']):.2f}")

        if "tmp1" in status:
            self.tmp1_label.setText(f"{float(status['tmp1']):.0f}")
        if "tmp2" in status:
            self.tmp2_label.setText(f"{float(status['tmp2']):.0f}")

        if "err1" in status:
            self.err1_label.setText(str(int(status["err1"])))
        if "err2" in status:
            self.err2_label.setText(str(int(status["err2"])))

    def closeEvent(self, event: QtGui.QCloseEvent):
        self._reader_stop = True
        with self._sim_lock:
            if self._sim_thread and self._sim_thread.is_alive():
                self._sim_stop.set()
        try:
            if self.ser and not isinstance(self.ser, DummySerial):
                self.ser.close()
        except Exception:
            pass

        try:
            if self._log_fp:
                self._log_fp.write(f"\nSerial Log Ended: {datetime.now().isoformat(timespec='seconds')}\n")
                self._log_fp.close()
                self._log_fp = None
        except Exception:
            pass
        super().closeEvent(event)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.resize(1100, 650)
    win.show()
    sys.exit(app.exec())
