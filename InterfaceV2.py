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
KT_NM_PER_A_1          = 0.08   # Motor 1 (Elbow)
KT_NM_PER_A_2          = 0.08   # Motor 2 (Wrist)

# Drawing sizes
ELBOW_L_UPPER          = 110  # px
ELBOW_L_FOREARM        = 110  # px
WRIST_DIAL_R           = 70   # px
WRIST_DIAL_OFFSET_DEG  = 90   # rotate drawing so numeric 0° points RIGHT visually


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
            "Elbow Flexion"
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
            "Wrist Supination"
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
    # use "object" instead of "dict" for cross-thread safety
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

        # Parameters (for command buttons)
        self.params = dict(
            rpm=0.0, pos=0.0, torque=0.0,
            duty=0.0, current=0.0, brake=0.0,
            posspd_p=0.0, posspd_v=0.0, posspd_a=0.0,
            aan_s=0.0, aan_e=0.0, aan_d=5.0
        )

        # Simulation handles
        self._sim_stop = threading.Event()
        self._sim_thread = None
        self._sim_lock = threading.Lock()

        # Reader threads control
        self._reader_stop = False

        self._build_ui()

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

        btn_origin = QtWidgets.QPushButton("Set Origin")
        btn_origin.setStyleSheet("background-color: #1976d2; color: white;")
        btn_origin.clicked.connect(lambda: self.send_cmd("ORIGIN"))
        left_layout.addWidget(btn_origin, 1, 0, 1, 3)

        btn_calib = QtWidgets.QPushButton("Calibrate")
        btn_calib.setStyleSheet("background-color: #ff9800; color: white;")
        btn_calib.clicked.connect(lambda: self.send_cmd("CALIBRATE"))
        left_layout.addWidget(btn_calib, 2, 0, 1, 3)

        btn_stop = QtWidgets.QPushButton("STOP")
        btn_stop.setStyleSheet("background-color: #d32f2f; color: white; font-weight:bold;")
        btn_stop.clicked.connect(self.stop_all)
        left_layout.addWidget(btn_stop, 3, 0, 1, 3)

        # Headers
        lbl_test = QtWidgets.QLabel("TEST MODES")
        lbl_test.setStyleSheet("font-weight:bold;")
        left_layout.addWidget(lbl_test, 4, 0, 1, 1)

        vline = QtWidgets.QFrame()
        vline.setFrameShape(QtWidgets.QFrame.VLine)
        vline.setFrameShadow(QtWidgets.QFrame.Sunken)
        left_layout.addWidget(vline, 4, 1, 9, 1)

        lbl_work = QtWidgets.QLabel("WORK MODES")
        lbl_work.setStyleSheet("font-weight:bold;")
        left_layout.addWidget(lbl_work, 4, 2, 1, 1)

        # Test mode buttons (left)
        row = 5
        btn_rpm = QtWidgets.QPushButton("RPM")
        btn_rpm.clicked.connect(lambda: self.send_cmd(f"RPM {int(self.params['rpm'])}"))
        left_layout.addWidget(btn_rpm, row, 0); row += 1

        btn_pos = QtWidgets.QPushButton("POS")
        btn_pos.clicked.connect(lambda: self.send_cmd(f"POS {self.params['pos']:.2f}"))
        left_layout.addWidget(btn_pos, row, 0); row += 1

        btn_torque = QtWidgets.QPushButton("TORQUE")
        btn_torque.clicked.connect(lambda: self.send_cmd(f"TORQUE {self.params['torque']:.2f}"))
        left_layout.addWidget(btn_torque, row, 0); row += 1

        btn_duty = QtWidgets.QPushButton("DUTY")
        btn_duty.clicked.connect(lambda: self.send_cmd(f"DUTY {self.params['duty']:.3f}"))
        left_layout.addWidget(btn_duty, row, 0); row += 1

        btn_current = QtWidgets.QPushButton("CURRENT")
        btn_current.clicked.connect(lambda: self.send_cmd(f"CURRENT {self.params['current']:.2f}"))
        left_layout.addWidget(btn_current, row, 0); row += 1

        btn_brake = QtWidgets.QPushButton("BRAKE")
        btn_brake.clicked.connect(lambda: self.send_cmd(f"BRAKE {self.params['brake']:.2f}"))
        left_layout.addWidget(btn_brake, row, 0); row += 1

        btn_posspd = QtWidgets.QPushButton("POSSPD")
        btn_posspd.clicked.connect(
            lambda: self.send_cmd(
                f"POSSPD {self.params['posspd_p']:.2f} "
                f"{self.params['posspd_v']:.2f} "
                f"{self.params['posspd_a']:.2f}"
            )
        )
        left_layout.addWidget(btn_posspd, row, 0); row += 1

        # Work mode buttons (right column)
        row_work = 5
        btn_aan = QtWidgets.QPushButton("Assist A/N")
        btn_aan.clicked.connect(self.open_aan_dialog)
        left_layout.addWidget(btn_aan, row_work, 2); row_work += 1

        btn_auto = QtWidgets.QPushButton("Auto Move")
        btn_auto.clicked.connect(lambda: self.send_cmd("POSSPD 90.00 18.00 1.00"))
        left_layout.addWidget(btn_auto, row_work, 2); row_work += 1

        btn_resist = QtWidgets.QPushButton("Resist")
        btn_resist.clicked.connect(lambda: self.send_cmd(f"TORQUE {self.params['torque']:.2f}"))
        left_layout.addWidget(btn_resist, row_work, 2); row_work += 1

        btn_sim = QtWidgets.QPushButton("Simulate Move")
        btn_sim.clicked.connect(self.open_sim_dialog)
        left_layout.addWidget(btn_sim, row_work, 2); row_work += 1

        left_layout.setRowStretch(row_work + 1, 1)

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

        status_layout.addWidget(QtWidgets.QLabel("Speed (ERPM):"), row_s, 0, alignment=QtCore.Qt.AlignRight)
        status_layout.addWidget(self.spd1_label, row_s, 1)
        status_layout.addWidget(self.spd2_label, row_s, 2); row_s += 1

        status_layout.addWidget(QtWidgets.QLabel("Torque (Nm):"), row_s, 0, alignment=QtCore.Qt.AlignRight)
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

    # ---------- Settings dialog ----------
    def open_settings(self):
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("Settings")
        layout = QtWidgets.QFormLayout(dlg)

        fields = [
            ("RPM", 'rpm'),
            ("Pos (°)", 'pos'),
            ("Torque (Nm)", 'torque'),
            ("Duty", 'duty'),
            ("Current (A)", 'current'),
            ("Brake (A)", 'brake'),
            ("PosSpd Pos (°)", 'posspd_p'),
            ("PosSpd Vel (deg/s)", 'posspd_v'),
            ("PosSpd Acc (deg/s²)", 'posspd_a'),
            ("AAN Start (°)", "aan_s"),
            ("AAN End (°)", "aan_e"),
            ("AAN Dur (s)", "aan_d"),
        ]


        widgets = {}
        for label, key in fields:
            le = QtWidgets.QLineEdit()
            le.setText(str(self.params[key]))
            layout.addRow(label + ":", le)
            widgets[key] = le

        btn_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel,
            parent=dlg
        )
        layout.addRow(btn_box)

        def on_ok():
            try:
                for key, le in widgets.items():
                    self.params[key] = float(le.text())
            except ValueError:
                QtWidgets.QMessageBox.warning(dlg, "Invalid input", "Please enter numeric values.")
                return
            dlg.accept()

        btn_box.accepted.connect(on_ok)
        btn_box.rejected.connect(dlg.reject)
        dlg.exec()

    # ---------- AAN dialog ----------
    def open_aan_dialog(self):
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("Assist-As-Needed Parameters")
        layout = QtWidgets.QFormLayout(dlg)

        # Joint selector
        joint_combo = QtWidgets.QComboBox()
        joint_combo.addItem("Elbow (Motor 1)", userData="AAN")
        joint_combo.addItem("Wrist (Motor 2)", userData="AAN2")
        layout.addRow("Joint:", joint_combo)

        le_start = QtWidgets.QLineEdit(str(self.params['aan_s']))
        le_end   = QtWidgets.QLineEdit(str(self.params['aan_e']))
        le_dur   = QtWidgets.QLineEdit(str(self.params['aan_d']))

        layout.addRow("Start (°):", le_start)
        layout.addRow("End (°):",   le_end)
        layout.addRow("Duration (s):", le_dur)

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

            self.params['aan_s'] = s
            self.params['aan_e'] = e
            self.params['aan_d'] = d

            cmd_name = joint_combo.currentData()  # "AAN" or "AAN2"
            self.send_cmd(f"{cmd_name} {s:.2f} {e:.2f} {d:.2f}")
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
    def stop_all(self):
        self.send_cmd("RPM 0")
        self.send_cmd("POS 0.00")
        self.send_cmd("TORQUE 0.00")
        self.send_cmd("POSSPD 0.00 0.00 0.00")

    def send_cmd(self, cmd: str):
        try:
            self.ser.write((cmd + "\n").encode())
        except Exception:
            pass
        # Mark outgoing commands
        self.log_signal.emit(f"TX: {cmd}")

    # ---------- Serial read + status parse ----------
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
                        # Mark incoming lines
                        self.log_signal.emit(f"RX: {txt}")
                        status = self.parse_status_line(txt)
                        if status:
                            self.status_signal.emit(status)
                    except Exception as e:
                        # Do not crash the GUI because of a bad line
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
        lo = line.lower()

        if not any(k in lo for k in [
            "pos", "pos1", "pos2", "elbow", "wrist", "sup", "deg", "deg1", "deg2",
            "spd", "spd1", "spd2", "erpm", "erpm1", "erpm2",
            "iq", "iq1", "iq2", "cur", "cur1", "cur2", "current",
            "torq", "torque", "tq", "temp", "temp1", "temp2", "err", "err1", "err2"
        ]):
            return None

        def pick(keys, default=None, cast=float):
            for k in keys:
                m = re.search(rf'\b{k}\s*[:= ]\s*(-?\d+(?:\.\d+)?)', lo)
                if m:
                    try:
                        return cast(m.group(1))
                    except Exception:
                        pass
            return default

        elbow_deg = pick(["elbow", "pos1", "p1", "deg1", "deg"])
        wrist_deg = pick(["wrist", "sup", "pos2", "p2", "deg2"])

        pos1 = pick(["pos1", "p1"])
        pos2 = pick(["pos2", "p2"])
        if elbow_deg is None:
            elbow_deg = pos1
        if wrist_deg is None:
            wrist_deg = pos2

        spd1 = pick(["spd1", "erpm1", "v1", "speed1", "spd", "erpm", "speed"])
        spd2 = pick(["spd2", "erpm2", "v2", "speed2"])

        iq1 = pick(["iq1", "cur1", "current1", "i1", "iq", "cur", "current"])
        iq2 = pick(["iq2", "cur2", "current2", "i2"])

        t1 = pick(["torque1", "torq1", "tq1", "t1", "torque"])
        t2 = pick(["torque2", "torq2", "tq2", "t2"])

        tmp1 = pick(["temp1", "t1c", "temperature1", "temp"])
        tmp2 = pick(["temp2", "t2c", "temperature2"])

        err1 = pick(["err1", "error1", "e1", "err", "error"], cast=int)
        err2 = pick(["err2", "error2", "e2"], cast=int)

        if t1 is None and iq1 is not None:
            t1 = float(iq1) * KT_NM_PER_A_1
        if t2 is None and iq2 is not None:
            t2 = float(iq2) * KT_NM_PER_A_2

        status = {}
        if elbow_deg is not None:
            status["elbow_deg"] = float(elbow_deg)
        if wrist_deg is not None:
            status["wrist_deg"] = max(-90.0, min(90.0, float(wrist_deg)))
        if spd1 is not None:
            status["spd1"] = float(spd1)
        if spd2 is not None:
            status["spd2"] = float(spd2)
        if t1 is not None:
            status["t1"] = float(t1)
        if t2 is not None:
            status["t2"] = float(t2)
        if tmp1 is not None:
            status["tmp1"] = float(tmp1)
        if tmp2 is not None:
            status["tmp2"] = float(tmp2)
        if err1 is not None:
            status["err1"] = int(err1)
        if err2 is not None:
            status["err2"] = int(err2)

        return status or None

    @QtCore.Slot(object)
    def apply_status_update(self, status: object):
        # status is expected to be a dict
        if not isinstance(status, dict):
            return
        if "elbow_deg" in status:
            val = status["elbow_deg"]
            self.elbow_view.set_angle_deg(val)
            self.pos1_label.setText(f"{val:.1f}")
        if "wrist_deg" in status:
            val = status["wrist_deg"]
            self.wrist_view.set_angle_deg(val)
            self.pos2_label.setText(f"{val:.1f}")
        if "spd1" in status:
            self.spd1_label.setText(f"{status['spd1']:.0f}")
        if "spd2" in status:
            self.spd2_label.setText(f"{status['spd2']:.0f}")
        if "t1" in status:
            self.trq1_label.setText(f"{status['t1']:.2f}")
        if "t2" in status:
            self.trq2_label.setText(f"{status['t2']:.2f}")
        if "tmp1" in status:
            self.tmp1_label.setText(f"{status['tmp1']:.0f}")
        if "tmp2" in status:
            self.tmp2_label.setText(f"{status['tmp2']:.0f}")
        if "err1" in status:
            self.err1_label.setText(str(status["err1"]))
        if "err2" in status:
            self.err2_label.setText(str(status["err2"]))

    # ---------- Close handling ----------
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
        super().closeEvent(event)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.resize(1000, 600)
    win.show()
    sys.exit(app.exec())
