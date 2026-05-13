GLOBAL_STYLE = """
/* Standard CMD Font and sharp edges for everything */
* {
    font-family: "Consolas", "Lucida Console", "Courier New", monospace;
    font-size: 13px;
    letter-spacing: 0px;
}

QMainWindow {
    background-color: #000000; /* Pure Black */
}

/* Sharp, high-contrast Tab Bar */
QTabBar::tab {
    background: #000000;
    color: #cccccc;
    border: 1px solid #333333; /* Dim gray border */
    border-bottom: none;
    padding: 5px 15px;
    margin-right: 1px;
}
QTabBar::tab:selected {
    background: #000000;
    color: #ffffff;
    border: 1px solid #ffffff; /* White border for selected tab */
    border-bottom: none;
}

/* Sharp borders for containers */
QTabWidget::pane {
    border: 1px solid #ffffff;
    background-color: #000000;
}

/* CMD Style Tables */
QTableWidget {
    background-color: #000000;
    color: #cccccc;
    gridline-color: #333333;
    border: 1px solid #333333;
}

/* CMD Style Input Box at the bottom */
QLineEdit {
    background-color: #000000;
    color: #ffffff;
    border: 1px solid #ffffff; /* Sharp white border */
    padding: 5px;
}

QPlainTextEdit {
    background-color: #000000;
    color: #ffffff;
    border: 1px solid #ffffff;
    /* ASCII CRITICAL FIXES */
    font-family: "Consolas", monospace;
    white-space: pre;
    line-height: 100%;
}
"""

import time
import collections

from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTabWidget,
                             QTableWidget, QTableWidgetItem, QPlainTextEdit,
                             QLineEdit, QHeaderView, QApplication)
from PyQt6.QtCore import pyqtSignal, QObject, pyqtSlot, QTimer, Qt, QEvent
from PyQt6.QtGui import QColor, QBrush, QFont, QIcon, QTextCharFormat, QTextCursor
from PyQt6 import uic 
from pathlib import Path
import pyqtgraph as pg

import parsestuff
from backend import SerialMonitor, MariDrone, list_serial_ports, is_connected, art, title
import controller

WATCHDOG_TIMEOUT = 1800
DISPLAY_MAX_LINES = 100
PING_TIMEOUT = 3

class UISignals(QObject):
    new_log_line = pyqtSignal(str)
    new_status = pyqtSignal(tuple)
    new_rtos = pyqtSignal(object)  # carries parsestuff.RTOS_Status

class FastLoRaGUI(QMainWindow):
    def __init__(self, mon: SerialMonitor, mari: MariDrone):
        super().__init__()
        self.mon = mon
        self.mari = mari
        self.signals = UISignals()
        
        self.timer_running = False
        self.start_time = 0.0
        self.elapsed_before_stop = 0.0
        self._display_counter = 0
        self._dt_window = collections.deque(maxlen=50)
        
        # Load the UI from the .ui file created with Qt Designer
        
        uic.loadUi(str(Path(__file__).with_name("lora_gui.ui")), self)
        self.setWindowIcon(QIcon(str(Path(__file__).with_name("mari.ico"))))
        
        # Command History tracking
        self.cmd_history = []
        self.cmd_index = 0
        
        # Tell the window to listen for key presses inside the cmd_input box
        self.cmd_input.installEventFilter(self)
        
        # Apply global stylesheet for asthetics
        self.setStyleSheet(GLOBAL_STYLE)
        
        # Set window title (Overrides the Designer title)
        self.setWindowTitle("MARI LoRa Monitor")
        
        # Initialize plot data structures
        self.plot_history_size = 200
        self.plot_time = collections.deque(maxlen=self.plot_history_size)
        self.plot_start_time = time.time()
        
        self.plot_pitch = collections.deque(maxlen=self.plot_history_size)
        self.plot_roll = collections.deque(maxlen=self.plot_history_size)
        self.plot_yaw = collections.deque(maxlen=self.plot_history_size)
        
        self.plot_mNW = collections.deque(maxlen=self.plot_history_size)
        self.plot_mNE = collections.deque(maxlen=self.plot_history_size)
        self.plot_mSW = collections.deque(maxlen=self.plot_history_size)
        self.plot_mSE = collections.deque(maxlen=self.plot_history_size)

        self.plot_pid_pitch = collections.deque(maxlen=self.plot_history_size)
        self.plot_pid_roll = collections.deque(maxlen=self.plot_history_size)
        self.plot_pid_yaw = collections.deque(maxlen=self.plot_history_size)
        
        self.plot_pitchP = collections.deque(maxlen=self.plot_history_size)
        self.plot_pitchI = collections.deque(maxlen=self.plot_history_size)
        self.plot_pitchD = collections.deque(maxlen=self.plot_history_size)
        
        self.plot_rollP = collections.deque(maxlen=self.plot_history_size)
        self.plot_rollI = collections.deque(maxlen=self.plot_history_size)
        self.plot_rollD = collections.deque(maxlen=self.plot_history_size)
        
        self.plot_yawP = collections.deque(maxlen=self.plot_history_size)
        self.plot_yawI = collections.deque(maxlen=self.plot_history_size)
        self.plot_yawD = collections.deque(maxlen=self.plot_history_size)

        # Fine-tuning buffers (Q + F frames). Attitude and motors reuse the
        # existing plot_pitch/roll/yaw and plot_mNW/NE/SW/SE buffers so the
        # fine-tuning panes mirror the IMU and Motor graphs exactly.
        self.plot_pitch_user = collections.deque(maxlen=self.plot_history_size)
        self.plot_roll_user = collections.deque(maxlen=self.plot_history_size)
        self.plot_yaw_user = collections.deque(maxlen=self.plot_history_size)
        self.plot_pitchError = collections.deque(maxlen=self.plot_history_size)
        self.plot_rollError = collections.deque(maxlen=self.plot_history_size)
        self.plot_yawError = collections.deque(maxlen=self.plot_history_size)

        # LoRa Buffers
        self.plot_rssi = collections.deque(maxlen=self.plot_history_size)
        self.plot_snr = collections.deque(maxlen=self.plot_history_size)

        # Initialize remaining UI elements (Graphs, Table headers)
        self._init_ui()
        self._connect_signals()

        # Injecting dependencies into the Controller via Callbacks
        self.controller = controller.CommandController(
            mon=self.mon, 
            mari=self.mari, 
            print_cb=self._log_lines, 
            clear_cb=self._clear_log, 
            quit_cb=QApplication.quit,
            stopwatch_cb=self._handle_stopwatch
        )
        
        self.tick_timer = QTimer(self)
        self.tick_timer.timeout.connect(self._tick)
        self.tick_timer.start(250)
        
        self.stopwatch_timer = QTimer(self)
        self.stopwatch_timer.timeout.connect(self._update_stopwatch_ui)
        self.stopwatch_timer.start(100)

        self._log_lines(art)
        self._log_lines("-------------------------------------------------")
        self._log_lines(title)
        self._log_lines("-------------------------------------------------\n")
        self._log_lines("GUI initialized. Type 'help' for commands.")

    def _init_ui(self):
        # Configure PyQtGraph globally
        pg.setConfigOption('background', '#000000') # Pure Black
        pg.setConfigOption('foreground', '#ffffff') # Pure White text/axes
        
        
        # Connect user input
        self.cmd_input.returnPressed.connect(self.on_input_submitted)
        
        # Force a true fixed-pitch font so ASCII art aligns column-by-column
        mono = QFont("Consolas")
        mono.setStyleHint(QFont.StyleHint.Monospace)
        mono.setFixedPitch(True)
        mono.setPointSize(10)
        self.log_widget.setFont(mono)
        self.log_widget.document().setDefaultFont(mono)
        
        # Set Table Header resize behaviors
        for t in [self.table, self.table_sensors, self.table_pids, self.table_cpu]:
            t.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
            t.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
            t.verticalHeader().setVisible(False)
            t.setColumnCount(2)
            t.setHorizontalHeaderLabels(["Metric", ""])
        
        self._setup_tables()
        
        # --- INJECT LORA GRAPHS INTO lora_graphs_container ---
        self.plt_rssi = pg.PlotWidget(title="RSSI (dBm)")
        self.plt_rssi.setLabel('left', 'dBm')
        self.plt_rssi.setLabel('bottom', 'Time (s)')
        self.line_rssi = self.plt_rssi.plot(pen=pg.mkPen('#FF5555', width=2), name="RSSI")
        self.lora_graphs_container.layout().addWidget(self.plt_rssi)
        
        self.plt_snr = pg.PlotWidget(title="SNR (dB)")
        self.plt_snr.setLabel('left', 'dB')
        self.plt_snr.setLabel('bottom', 'Time (s)')
        self.line_snr = self.plt_snr.plot(pen=pg.mkPen('#55FFFF', width=2), name="SNR")
        self.lora_graphs_container.layout().addWidget(self.plt_snr)
        
        # --- INJECT IMU AND MOTOR GRAPHS INTO verticalLayout_2 ---
        top_sensor_graphs_layout = QHBoxLayout()
        
        # Create a vertical layout to stack the 3 separate IMU graphs
        imu_layout = QVBoxLayout()
        
        # 1. Pitch Plot (-180 to 180)
        self.plt_pitch = pg.PlotWidget(title="IMU Pitch")
        self.plt_pitch.setYRange(-180, 180, padding=0)
        self.plt_pitch.setLabel('left', 'Degrees')
        self.plt_pitch.setLabel('bottom', 'Time (s)')
        self.plt_pitch.showGrid(x=True, y=True, alpha=0.5)
        self.line_pitch = self.plt_pitch.plot(pen=pg.mkPen('#FF5555', width=2), name="Pitch")
        imu_layout.addWidget(self.plt_pitch)
        
        # 2. Roll Plot (-90 to 90)
        self.plt_roll = pg.PlotWidget(title="IMU Roll")
        self.plt_roll.setYRange(-90, 90, padding=0)
        self.plt_roll.setLabel('left', 'Degrees')
        self.plt_roll.setLabel('bottom', 'Time (s)')
        self.plt_roll.showGrid(x=True, y=True, alpha=0.5)
        self.line_roll = self.plt_roll.plot(pen=pg.mkPen('#55FF55', width=2), name="Roll")
        imu_layout.addWidget(self.plt_roll)
        
        # 3. Yaw Plot (0 to 360)
        self.plt_yaw = pg.PlotWidget(title="IMU Yaw")
        self.plt_yaw.setYRange(-180, 180, padding=0)
        self.plt_yaw.setLabel('left', 'Degrees')
        self.plt_yaw.setLabel('bottom', 'Time (s)')
        self.plt_yaw.showGrid(x=True, y=True, alpha=0.5)
        self.line_yaw = self.plt_yaw.plot(pen=pg.mkPen('#55FFFF', width=2), name="Yaw")
        imu_layout.addWidget(self.plt_yaw)
        
        # Add the stacked IMU graphs to the left side
        top_sensor_graphs_layout.addLayout(imu_layout)
        
        # --- Motor Outputs (Right side) ---
        self.plt_motors = pg.PlotWidget(title="Motor Outputs")
        self.plt_motors.setYRange(900, 2000, padding=0)
        self.plt_motors.setLabel('left', 'PWM Value')
        self.plt_motors.setLabel('bottom', 'Time (s)')
        self.plt_motors.addLegend()
        self.plt_motors.showGrid(x=True, y=True, alpha=0.5)
        
        self.line_mNW = self.plt_motors.plot(pen=pg.mkPen('#FF5555', width=2), name="NW")
        self.line_mNE = self.plt_motors.plot(pen=pg.mkPen('#55FF55', width=2), name="NE")
        self.line_mSW = self.plt_motors.plot(pen=pg.mkPen('#55FFFF', width=2), name="SW")
        self.line_mSE = self.plt_motors.plot(pen=pg.mkPen('#FFFF55', width=2), name="SE")
        
        top_sensor_graphs_layout.addWidget(self.plt_motors)
        self.verticalLayout_2.addLayout(top_sensor_graphs_layout)
        
        # --- INJECT PID GRAPHS INTO verticalLayout_3 ---
        top_pid_graphs_layout = QHBoxLayout()
        bottom_pid_graphs_layout = QHBoxLayout()
        
        self.plt_pitch_pid = pg.PlotWidget(title="Pitch PID Components")
        self.plt_pitch_pid.setYRange(-100, 100, padding=0)
        self.plt_pitch_pid.addLegend()
        self.plt_pitch_pid.setLabel('left', 'PID Value')
        self.plt_pitch_pid.setLabel('bottom', 'Time (s)')
        self.plt_pitch_pid.showGrid(x=True, y=True, alpha=0.5)
        self.line_pitchP = self.plt_pitch_pid.plot(pen=pg.mkPen('#FFFF55', width=2), name="P")
        self.line_pitchI = self.plt_pitch_pid.plot(pen=pg.mkPen('#FF55FF', width=2), name="I")
        self.line_pitchD = self.plt_pitch_pid.plot(pen=pg.mkPen('#55FFFF', width=2), name="D")
        top_pid_graphs_layout.addWidget(self.plt_pitch_pid)
        
        self.plt_roll_pid = pg.PlotWidget(title="Roll PID Components")
        self.plt_roll_pid.setYRange(-100, 100, padding=0)
        self.plt_roll_pid.addLegend()
        self.plt_roll_pid.setLabel('left', 'PID Value')
        self.plt_roll_pid.setLabel('bottom', 'Time (s)')
        self.plt_roll_pid.showGrid(x=True, y=True, alpha=0.5)
        self.line_rollP = self.plt_roll_pid.plot(pen=pg.mkPen('#FFFF55', width=2), name="P")
        self.line_rollI = self.plt_roll_pid.plot(pen=pg.mkPen('#FF55FF', width=2), name="I")
        self.line_rollD = self.plt_roll_pid.plot(pen=pg.mkPen('#55FFFF', width=2), name="D")
        top_pid_graphs_layout.addWidget(self.plt_roll_pid)
        self.verticalLayout_3.addLayout(top_pid_graphs_layout)
        
        self.plt_yaw_pid = pg.PlotWidget(title="Yaw PID Components")
        self.plt_yaw_pid.setYRange(-100, 100, padding=0)
        self.plt_yaw_pid.addLegend()
        self.plt_yaw_pid.setLabel('left', 'PID Value')
        self.plt_yaw_pid.setLabel('bottom', 'Time (s)')
        self.plt_yaw_pid.showGrid(x=True, y=True, alpha=0.5)
        self.line_yawP = self.plt_yaw_pid.plot(pen=pg.mkPen('#FFFF55', width=2), name="P")
        self.line_yawI = self.plt_yaw_pid.plot(pen=pg.mkPen('#FF55FF', width=2), name="I")
        self.line_yawD = self.plt_yaw_pid.plot(pen=pg.mkPen('#55FFFF', width=2), name="D")
        bottom_pid_graphs_layout.addWidget(self.plt_yaw_pid)
        
        self.plt_pid = pg.PlotWidget(title="PID Totals")
        self.plt_pid.setYRange(-100, 100, padding=0)
        self.plt_pid.addLegend()
        self.plt_pid.setLabel('left', 'PID Total Value')
        self.plt_pid.setLabel('bottom', 'Time (s)')
        self.plt_pid.showGrid(x=True, y=True, alpha=0.5)
        self.line_pid_pitch = self.plt_pid.plot(pen=pg.mkPen('#FF5555', width=2), name="PID Pitch")
        self.line_pid_roll = self.plt_pid.plot(pen=pg.mkPen('#55FF55', width=2), name="PID Roll")
        self.line_pid_yaw = self.plt_pid.plot(pen=pg.mkPen('#55FFFF', width=2), name="PID Yaw")
        bottom_pid_graphs_layout.addWidget(self.plt_pid)
        self.verticalLayout_3.addLayout(bottom_pid_graphs_layout)

        # --- INJECT FINE-TUNING GRAPHS INTO verticalLayout_ft ---
        # Pane 1: Attitude (3 solid IMU + 3 dashed user setpoints)
        self.plt_ft_attitude = pg.PlotWidget(title="Attitude (solid) vs User Setpoint (dashed)")
        self.plt_ft_attitude.setYRange(-30, 30, padding=0)
        self.plt_ft_attitude.setLabel('left', 'Degrees')
        self.plt_ft_attitude.setLabel('bottom', 'Time (s)')
        self.plt_ft_attitude.addLegend()
        self.plt_ft_attitude.showGrid(x=True, y=True, alpha=0.5)
        self.line_ft_pitch = self.plt_ft_attitude.plot(pen=pg.mkPen('#FF5555', width=2), name="Pitch")
        self.line_ft_roll = self.plt_ft_attitude.plot(pen=pg.mkPen('#55FF55', width=2), name="Roll")
        self.line_ft_yaw = self.plt_ft_attitude.plot(pen=pg.mkPen('#55FFFF', width=2), name="Yaw")
        self.line_ft_pitch_user = self.plt_ft_attitude.plot(pen=pg.mkPen('#FF5555', width=1, style=Qt.PenStyle.DashLine), name="Pitch SP")
        self.line_ft_roll_user = self.plt_ft_attitude.plot(pen=pg.mkPen('#55FF55', width=1, style=Qt.PenStyle.DashLine), name="Roll SP")
        self.line_ft_yaw_user = self.plt_ft_attitude.plot(pen=pg.mkPen('#55FFFF', width=1, style=Qt.PenStyle.DashLine), name="Yaw SP")
        self.verticalLayout_ft.addWidget(self.plt_ft_attitude)

        # Pane 2: Error (oscillation indicator)
        self.plt_ft_error = pg.PlotWidget(title="Error (oscillation indicator)")
        self.plt_ft_error.setYRange(-15, 15, padding=0)
        self.plt_ft_error.setLabel('left', 'Error (deg)')
        self.plt_ft_error.setLabel('bottom', 'Time (s)')
        self.plt_ft_error.addLegend()
        self.plt_ft_error.showGrid(x=True, y=True, alpha=0.5)
        self.line_ft_pitchE = self.plt_ft_error.plot(pen=pg.mkPen('#FF5555', width=2), name="Pitch Error")
        self.line_ft_rollE = self.plt_ft_error.plot(pen=pg.mkPen('#55FF55', width=2), name="Roll Error")
        self.line_ft_yawE = self.plt_ft_error.plot(pen=pg.mkPen('#55FFFF', width=2), name="Yaw Error")
        self.verticalLayout_ft.addWidget(self.plt_ft_error)

        # Pane 3: I terms (windup) | D terms (noise)
        ft_id_layout = QHBoxLayout()

        self.plt_ft_i = pg.PlotWidget(title="I terms (windup indicator)")
        self.plt_ft_i.setYRange(-500, 500, padding=0)
        self.plt_ft_i.setLabel('left', 'I value')
        self.plt_ft_i.setLabel('bottom', 'Time (s)')
        self.plt_ft_i.addLegend()
        self.plt_ft_i.showGrid(x=True, y=True, alpha=0.5)
        self.line_ft_pitchI = self.plt_ft_i.plot(pen=pg.mkPen('#FF5555', width=2), name="Pitch I")
        self.line_ft_rollI = self.plt_ft_i.plot(pen=pg.mkPen('#55FF55', width=2), name="Roll I")
        self.line_ft_yawI = self.plt_ft_i.plot(pen=pg.mkPen('#55FFFF', width=2), name="Yaw I")
        ft_id_layout.addWidget(self.plt_ft_i)

        self.plt_ft_d = pg.PlotWidget(title="D terms (noise indicator)")
        self.plt_ft_d.setYRange(-50, 50, padding=0)
        self.plt_ft_d.setLabel('left', 'D value')
        self.plt_ft_d.setLabel('bottom', 'Time (s)')
        self.plt_ft_d.addLegend()
        self.plt_ft_d.showGrid(x=True, y=True, alpha=0.5)
        self.line_ft_pitchD = self.plt_ft_d.plot(pen=pg.mkPen('#FF5555', width=2), name="Pitch D")
        self.line_ft_rollD = self.plt_ft_d.plot(pen=pg.mkPen('#55FF55', width=2), name="Roll D")
        self.line_ft_yawD = self.plt_ft_d.plot(pen=pg.mkPen('#55FFFF', width=2), name="Yaw D")
        ft_id_layout.addWidget(self.plt_ft_d)
        self.verticalLayout_ft.addLayout(ft_id_layout)

        # Pane 4: Motors (saturation indicator) - corner colors NW/NE/SW/SE
        self.plt_ft_motors = pg.PlotWidget(title="Motors (saturation indicator)")
        self.plt_ft_motors.setYRange(980, 2000, padding=0)
        self.plt_ft_motors.setLabel('left', 'PWM Value')
        self.plt_ft_motors.setLabel('bottom', 'Time (s)')
        self.plt_ft_motors.addLegend()
        self.plt_ft_motors.showGrid(x=True, y=True, alpha=0.5)
        self.line_ft_m0 = self.plt_ft_motors.plot(pen=pg.mkPen('#FF5555', width=2), name="motor[0] NW")
        self.line_ft_m1 = self.plt_ft_motors.plot(pen=pg.mkPen('#55FF55', width=2), name="motor[1] NE")
        self.line_ft_m2 = self.plt_ft_motors.plot(pen=pg.mkPen('#55FFFF', width=2), name="motor[2] SW")
        self.line_ft_m3 = self.plt_ft_motors.plot(pen=pg.mkPen('#FFFF55', width=2), name="motor[3] SE")
        self.verticalLayout_ft.addWidget(self.plt_ft_motors)

        self.log_widget.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap) # Forces exact character placement
        self.log_widget.setTabStopDistance(4) # Ensures tabs don't warp alignment

    def eventFilter(self, source, event):
        # Catch key presses in the command input box
        if source == self.cmd_input and event.type() == QEvent.Type.KeyPress:
            if event.key() == Qt.Key.Key_Up:
                if self.cmd_history and self.cmd_index > 0:
                    self.cmd_index -= 1
                    self.cmd_input.setText(self.cmd_history[self.cmd_index])
                return True # Tell Qt we handled this event
                
            elif event.key() == Qt.Key.Key_Down:
                if self.cmd_history and self.cmd_index < len(self.cmd_history) - 1:
                    self.cmd_index += 1
                    self.cmd_input.setText(self.cmd_history[self.cmd_index])
                elif self.cmd_history and self.cmd_index == len(self.cmd_history) - 1:
                    # If we reach the bottom, clear the box
                    self.cmd_index = len(self.cmd_history)
                    self.cmd_input.clear()
                return True
                
        return super().eventFilter(source, event)

    def _setup_tables(self):
        lora_rows = [
            ("debug", "Debug Mode"), ("display", "Display RX"), ("conn", "Serial Connection"),
            ("port", "Port"), ("baud", "Baud"), ("rssi", "Last RSSI (dBm)"), ("rssi_checker", "RSSI Okay?"),
            ("snr", "Last SNR (dB)"), ("snr_checker", "SNR Okay?"), ("link_quality", "Link Quality"), ("spacer1", ""),("data", "Last Data"),("uuid", "Last UUID"),
            ("type", "Last Type"),("sequence", "Last Sequence"), ("payload", "Last Payload"), ("len", "Last Length (bytes)"), ("target", "MARI Target Addr"),
            ("spacer2", ""),("sunny_pdr", "MARI Ping PDR"),("imu_pdr", "IMU PDR"), ("pid_pdr", "PID PDR"), ("motor_pdr", "Motor PDR"), ("esc_pdr", "ESC PDR"), ("batt_pdr", "Battery PDR"), ("raw_pdr", "Raw PDR"), ("cpu_pdr", "CPU PDR"), ("rate_pdr", "Task Rate PDR")
        ]
        self.lora_row_map = {}
        for row, (key, label) in enumerate(lora_rows):
            self.table.insertRow(row)
            self.table.setItem(row, 0, QTableWidgetItem(label))
            self.table.setItem(row, 1, QTableWidgetItem("-"))
            self.lora_row_map[key] = row

        sensor_rows = [
            ("pitch", "Pitch (deg)"), ("roll", "Roll (deg)"), ("yaw", "Yaw (deg)"),
            ("temp", "Temperature (°C)"), ("pressure", "Pressure (hPa)"), ("alt", "Altitude (m)"),
            ("batt_volt", "Batt Volt (V)"), ("batt_status", "Battery Status"), ("sensor_spacer1", ""),
            ("sensor_spacer2", ""),
            ("motorNW", "Motor NW"), ("motorNE", "Motor NE"), ("motorSW", "Motor SW"), ("motorSE", "Motor SE"),
            ("sensor_spacer3", ""),
            ("dt_ms", "dt (ms)"), ("max_dt_ms", "max dt (ms)"),
            ("sensor_spacer4", ""), ("time", "Time")
        ]
        self.sensor_row_map = {}
        for row, (key, label) in enumerate(sensor_rows):
            self.table_sensors.insertRow(row)
            self.table_sensors.setItem(row, 0, QTableWidgetItem(label))
            self.table_sensors.setItem(row, 1, QTableWidgetItem("-"))
            self.sensor_row_map[key] = row

        pids_rows = [
            ("pitchP", "Pitch P"), ("pitchI", "Pitch I"), ("pitchD", "Pitch D"), 
            ("pids_spacer1", ""), ("rollP", "Roll P"), ("rollI", "Roll I"), ("rollD", "Roll D"),
            ("pids_spacer2", ""), ("yawP", "Yaw P"), ("yawI", "Yaw I"), ("yawD", "Yaw D"),
            ("pids_spacer3", ""), ("pitchPID", "Pitch PID (Total)"), ("rollPID", "Roll PID (Total)"), ("yawPID", "Yaw PID (Total)")
        ]
        self.pids_row_map = {}
        for row, (key, label) in enumerate(pids_rows):
            self.table_pids.insertRow(row)
            self.table_pids.setItem(row, 0, QTableWidgetItem(label))
            self.table_pids.setItem(row, 1, QTableWidgetItem("-"))
            self.pids_row_map[key] = row

        cpu_rows = [
            ("flight_rate", "Flight Rate (Hz)"), ("tele_tx_rate", "Tele TX Rate (Hz)"),
            ("tele_rx_rate", "Tele RX Rate (Hz)"), ("housekeeping_rate", "Housekeeping Rate (Hz)"),
            ("cpu_spacer1", ""),
            ("flight_cpu_per", "Flight CPU %"), ("tele_tx_cpu_per", "Tele TX CPU %"),
            ("tele_rx_cpu_per", "Tele RX CPU %"), ("housekeeping_cpu_per", "Housekeeping CPU %"),
            ("idle_cpu_per", "Idle CPU %"),
        ]
        self.cpu_row_map = {}
        for row, (key, label) in enumerate(cpu_rows):
            self.table_cpu.insertRow(row)
            self.table_cpu.setItem(row, 0, QTableWidgetItem(label))
            self.table_cpu.setItem(row, 1, QTableWidgetItem("-"))
            self.cpu_row_map[key] = row

    def _connect_signals(self):
        self.mon.set_output_cb(lambda line: self.signals.new_log_line.emit(line))
        self.mon.set_status_cb(lambda status: self.signals.new_status.emit(status))
        self.mon.set_rtos_cb(lambda rtos: self.signals.new_rtos.emit(rtos))
        self.signals.new_log_line.connect(self._log_lines)
        self.signals.new_status.connect(self._handle_status_update)
        self.signals.new_rtos.connect(self._handle_rtos_update)

    @pyqtSlot(str)
    def _log_lines(self, s: str):
        # Use a cursor to insert text without extra paragraph padding
        cursor = self.log_widget.textCursor()
        cursor.movePosition(cursor.MoveOperation.End)
        
        color = QColor("#FFFFFF") # Default: bright white
        lower_s = s.lower()

        # Determine color based on keywords (bright ANSI palette)
        if "error:" in lower_s or "failure" in lower_s or "timeout" in lower_s:
            color = QColor("#FF5555") # Bright Red
        elif "system:" in lower_s:
            color = QColor("#5555FF") # Bright Blue
        elif "mari:" in lower_s:
            color = QColor("#FF55FF") # Bright Magenta
        elif "rx:" in lower_s or "success" in lower_s:
            color = QColor("#55FF55") # Bright Green
        elif "tx:" in lower_s or s.startswith(">"):
            color = QColor("#FFFF55") # Bright Yellow

        fmt = QTextCharFormat()
        fmt.setForeground(QBrush(color))

        # Determine if we need a newline (don't add one for the very first line)
        prefix = "\n" if self.log_widget.toPlainText() else ""
        cursor.insertText(prefix + s, fmt)
        
        self._display_counter += 1
        
        # Auto-scroll
        scrollbar = self.log_widget.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        
        if self._display_counter > DISPLAY_MAX_LINES:
            self._clear_log()
            self._log_lines(f"system: Log auto-cleared after {DISPLAY_MAX_LINES} lines.")

    def _clear_log(self):
        # 1. Clear the Console Text
        self.log_widget.clear()
        self._display_counter = 0

        # 2. Reset the Time Reference
        self.plot_start_time = time.time()
        
        # 3. Clear all deques (data buffers)
        # We loop through all attributes of the class and clear any that are deques
        for attr_name in dir(self):
            attr = getattr(self, attr_name)
            if isinstance(attr, collections.deque):
                attr.clear()

        # 4. Refresh the Plot Lines immediately
        # This prevents the "old" lines from staying on screen until the next tick
        self._reset_plot_lines()
        
        self._log_lines("system: Console and graph buffers cleared.")

    def _reset_plot_lines(self):
        """Forces all graph lines to empty data sets."""
        empty_list = []
        # LoRa Lines
        self.line_rssi.setData(empty_list, empty_list)
        self.line_snr.setData(empty_list, empty_list)
        # IMU Lines
        self.line_pitch.setData(empty_list, empty_list)
        self.line_roll.setData(empty_list, empty_list)
        self.line_yaw.setData(empty_list, empty_list)
        # Motor Lines
        self.line_mNW.setData(empty_list, empty_list)
        self.line_mNE.setData(empty_list, empty_list)
        self.line_mSW.setData(empty_list, empty_list)
        self.line_mSE.setData(empty_list, empty_list)
        # PID Lines
        self.line_pid_pitch.setData(empty_list, empty_list)
        self.line_pid_roll.setData(empty_list, empty_list)
        self.line_pid_yaw.setData(empty_list, empty_list)
        
        # Pitch Lines
        self.line_pitchD.setData(empty_list, empty_list)
        self.line_pitchI.setData(empty_list, empty_list)
        self.line_pitchP.setData(empty_list, empty_list)
        
        # Roll Lines
        self.line_rollD.setData(empty_list, empty_list)
        self.line_rollI.setData(empty_list, empty_list)
        self.line_rollP.setData(empty_list, empty_list)
        
        # Yaw Lines
        self.line_yawD.setData(empty_list, empty_list)
        self.line_yawI.setData(empty_list, empty_list)
        self.line_yawP.setData(empty_list, empty_list)

        # Fine-tuning Lines
        self.line_ft_pitch.setData(empty_list, empty_list)
        self.line_ft_roll.setData(empty_list, empty_list)
        self.line_ft_yaw.setData(empty_list, empty_list)
        self.line_ft_pitch_user.setData(empty_list, empty_list)
        self.line_ft_roll_user.setData(empty_list, empty_list)
        self.line_ft_yaw_user.setData(empty_list, empty_list)
        self.line_ft_pitchE.setData(empty_list, empty_list)
        self.line_ft_rollE.setData(empty_list, empty_list)
        self.line_ft_yawE.setData(empty_list, empty_list)
        self.line_ft_pitchI.setData(empty_list, empty_list)
        self.line_ft_rollI.setData(empty_list, empty_list)
        self.line_ft_yawI.setData(empty_list, empty_list)
        self.line_ft_pitchD.setData(empty_list, empty_list)
        self.line_ft_rollD.setData(empty_list, empty_list)
        self.line_ft_yawD.setData(empty_list, empty_list)
        self.line_ft_m0.setData(empty_list, empty_list)
        self.line_ft_m1.setData(empty_list, empty_list)
        self.line_ft_m2.setData(empty_list, empty_list)
        self.line_ft_m3.setData(empty_list, empty_list)




    def _handle_stopwatch(self, action: str):
        if action == "start":
            if not self.timer_running:
                self.start_time = time.perf_counter() - self.elapsed_before_stop
                self.timer_running = True
                self._log_lines("system: Stopwatch started.")
        elif action == "stop":
            if self.timer_running:
                self.elapsed_before_stop = time.perf_counter() - self.start_time
                self.timer_running = False
                self._log_lines("system: Stopwatch stopped.")
        elif action == "reset":
            self.start_time = time.perf_counter()
            self.elapsed_before_stop = 0.0
            if not self.timer_running:
                self._update_stopwatch_ui()
            self._log_lines("system: Stopwatch reset.")

    def on_input_submitted(self):
        raw = self.cmd_input.text()
        self.cmd_input.clear()
        
        if raw.strip():
            # Prevent adding duplicate back-to-back commands
            if not self.cmd_history or self.cmd_history[-1] != raw:
                self.cmd_history.append(raw)
            
            # Reset the index to the end of the list
            self.cmd_index = len(self.cmd_history)
            
            self.controller.execute(raw)

    @pyqtSlot(tuple)
    def _handle_status_update(self, type_and_status: tuple[str, parsestuff.DroneStatus]):
        msg_type, status = type_and_status
        current = self.mari.sensor_stat
        
        if msg_type == "I":
            current.pitch = status.pitch
            current.roll = status.roll
            current.yaw = status.yaw
            current.temperature = status.temperature
            current.pressure = status.pressure
            current.altitude = status.altitude
            imu_pdr = self.mari.packet_stats["I"].pdr if "I" in self.mari.packet_stats else 0.0   
        elif msg_type == "P":
            current.pitchP = status.pitchP
            current.pitchI = status.pitchI
            current.pitchD = status.pitchD
            current.rollP = status.rollP
            current.rollI = status.rollI
            current.rollD = status.rollD
            current.yawP = status.yawP
            current.yawI = status.yawI
            current.yawD = status.yawD
            current.pitchPID = status.pitchPID
            current.rollPID = status.rollPID
            current.yawPID = status.yawPID
            
        elif msg_type == "M":
            current.motorNW = status.motorNW
            current.motorNE = status.motorNE
            current.motorSW = status.motorSW
            current.motorSE = status.motorSE
        elif msg_type == "E":
            current.motors = status.motors
        elif msg_type == "B":
            current.battery_status = status.battery_status
        elif msg_type == "RW" or msg_type == "MARI":
            current.raw_data = status.raw_data
        elif msg_type == "F":
            current.pitch = status.pitch
            current.roll = status.roll
            current.yaw = status.yaw
            current.pitchError = status.pitchError
            current.rollError = status.rollError
            current.yawError = status.yawError
            current.motors = status.motors
            # Mirror motors[0..3] into NW/NE/SW/SE so the existing motor graph
            # and the fine-tuning motors pane stay in sync.
            if status.motors and len(status.motors) >= 4:
                current.motorNW = status.motors[0]
                current.motorNE = status.motors[1]
                current.motorSW = status.motors[2]
                current.motorSE = status.motors[3]
            current.dt = status.dt
        elif msg_type == "Q":
            current.pitch_input = status.pitch_input
            current.roll_input = status.roll_input
            current.yaw_input = status.yaw_input
            current.pitchI = status.pitchI
            current.pitchD = status.pitchD
            current.rollI = status.rollI
            current.rollD = status.rollD
            current.yawI = status.yawI
            current.yawD = status.yawD
            current.battery_voltage = status.battery_voltage


        self.mari.sensor_stat = current

    @pyqtSlot(object)
    def _handle_rtos_update(self, rtos: parsestuff.RTOS_Status):
        # Backend has already written into self.mari.rtos_stat; keep the local
        # reference assigned so callers using this signal stay typesafe.
        # _tick reads from self.mari.rtos_stat each interval to refresh table_cpu cells.
        self.mari.rtos_stat = rtos

    def _update_cell(self, table: QTableWidget, row_map: dict, key: str, value: str, color_hex: str = None, bold: bool = False, italic: bool = False):
        row = row_map.get(key)
        if row is not None:
            item = table.item(row, 1)
            if not item:
                item = QTableWidgetItem()
                table.setItem(row, 1, item)
            
            item.setText(value)
            
            font = item.font()
            font.setBold(bold)
            font.setItalic(italic)
            item.setFont(font)
            
            if color_hex:
                item.setForeground(QBrush(QColor(color_hex)))
            else:
                item.setForeground(QBrush())

    def _tick(self):
        data_frame = parsestuff.parse_data_line(self.mon.data.decode(errors="replace"))
        parsed_type = data_frame.type if data_frame else "RW"
        parsed_payload = data_frame.payload if data_frame else ""
        parsed_uuid = data_frame.uuid if data_frame else "-"
        parsed_sequence = str(data_frame.sequence) if data_frame and hasattr(data_frame, 'sequence') else "-"
        last_rssi = parsestuff.rssi_okay(self.mari.rssi) if self.mari.rssi else "-"
        last_snr = parsestuff.snr_okay(self.mari.snr) if self.mari.snr else "-"
        last_link_quality = self.mari.link_quality if self.mari.link_quality else "-"
        
        current_time = time.perf_counter()
        for seq, send_time in list(self.mari.pending_sunnys.items()):
            if current_time - send_time > PING_TIMEOUT:  # 3.0 second timeout limit
                self.mari.ping_state = parsestuff.PingState.TIMEOUT
                self.mari.pending_sunnys.pop(seq)
                self._log_lines(f"system: error - SUN {seq} TIMEOUT. Handshake failed.")
                
        # --- UPDATE MARI STATUS UI ---
        state = self.mari.ping_state
        if state == parsestuff.PingState.WAITING:
            status_text = "WAITING..."
            status_color = "#FFFF55" # Bright Yellow
        elif state == parsestuff.PingState.CONNECTED:
            status_text = f"CONNECTED ({self.mari.sunny_rtt:.1f} ms)" if self.mari.sunny_rtt else "CONNECTED"
            status_color = "#55FF55" # Bright Green
        elif state == parsestuff.PingState.TIMEOUT:
            status_text = "TIMEOUT"
            status_color = "#FF5555" # Bright Red
        else:
            status_text = "DISCONNECTED"
            status_color = "#AAAAAA" # ANSI Light Gray

        # 1. Update HTML with terminal colors
        html_text = f"""
            <div style="margin: 2px;">
                <span style="color: #ffffff; font-weight: normal;">MARI STATUS: </span>
                <span style="color: {status_color}; font-weight: bold;">{status_text}</span>
            </div>
        """
        self.label_ping_status.setText(html_text)

        # 2. Change style to sharp, thin borders (CMD style)
        self.label_ping_status.setStyleSheet(f"""
            QLabel {{
                background-color: #000000;
                border: 1px solid #ffffff; /* Sharp white outline */
                border-radius: 0px;        /* NO ROUNDED CORNERS */
                padding: 4px 10px;
            }}
        """)
        
        pid_pdr = self.mari.packet_stats["P"].pdr if "P" in self.mari.packet_stats else 0.0
        user_pdr = self.mari.packet_stats["U"].pdr if "U" in self.mari.packet_stats else 0.0
        motor_pdr = self.mari.packet_stats["M"].pdr if "M" in self.mari.packet_stats else 0.0
        esc_pdr = self.mari.packet_stats["E"].pdr if "E" in self.mari.packet_stats else 0.0
        batt_pdr = self.mari.packet_stats["B"].pdr if "B" in self.mari.packet_stats else 0.0
        raw_pdr = self.mari.packet_stats["RW"].pdr if "RW" in self.mari.packet_stats else 0.0
        sunny_pdr = self.mari.packet_stats["MARI"].pdr if "MARI" in self.mari.packet_stats else 0.0
        cpu_pdr = self.mari.packet_stats["C"].pdr if "C" in self.mari.packet_stats else 0.0
        rate_pdr = self.mari.packet_stats["R"].pdr if "R" in self.mari.packet_stats else 0.0

        self._update_cell(self.table, self.lora_row_map, "sunny_pdr", f"{sunny_pdr:.1%}")
        self._update_cell(self.table, self.lora_row_map, "imu_pdr", f"{user_pdr:.1%}")
        self._update_cell(self.table, self.lora_row_map, "pid_pdr", f"{pid_pdr:.1%}")
        self._update_cell(self.table, self.lora_row_map, "motor_pdr", f"{motor_pdr:.1%}")
        self._update_cell(self.table, self.lora_row_map, "esc_pdr", f"{esc_pdr:.1%}")
        self._update_cell(self.table, self.lora_row_map, "batt_pdr", f"{batt_pdr:.1%}")
        self._update_cell(self.table, self.lora_row_map, "raw_pdr", f"{raw_pdr:.1%}")
        self._update_cell(self.table, self.lora_row_map, "cpu_pdr", f"{cpu_pdr:.1%}")
        self._update_cell(self.table, self.lora_row_map, "rate_pdr", f"{rate_pdr:.1%}")

        if is_connected(self.mon):
            self._update_cell(self.table, self.lora_row_map, "conn", "CONNECTED", color_hex="#55FF55", bold=True)
        elif self.mon.debug:
            self._update_cell(self.table, self.lora_row_map, "conn", "DEBUG", color_hex="#FFFF55", bold=True, italic=self.mon.debug)
        else:
            self._update_cell(self.table, self.lora_row_map, "conn", "DISCONNECTED", color_hex="#FF5555", bold=True)
            
        self._update_cell(self.table, self.lora_row_map, "debug", "ON" if self.mon.debug else "OFF", color_hex="#FFFF55" if self.mon.debug else "#55FF55", bold=self.mon.debug, italic=self.mon.debug)
        self._update_cell(self.table, self.lora_row_map, "rssi", str(self.mari.rssi))
        self._update_cell(self.table, self.lora_row_map, "rssi_checker", last_rssi, color_hex="#55FF55" if last_rssi in ["Strong"] else "#FFFF55" if last_rssi in ["Fair"] else "#FF5555", bold=True)
        self._update_cell(self.table, self.lora_row_map, "snr", str(self.mari.snr))
        self._update_cell(self.table, self.lora_row_map, "snr_checker", last_snr, color_hex="#55FF55" if last_snr in ["Strong", "Good"] else "#FFFF55" if last_snr in ["Fair"] else "#FF5555", bold=True)
        self._update_cell(self.table, self.lora_row_map, "link_quality", last_link_quality, color_hex="#55FF55" if last_link_quality in ["Strong", "Good"] else "#FFFF55" if last_link_quality in ["Fair"] else "#FF5555", bold=True)
        self._update_cell(self.table, self.lora_row_map, "port", self.mon.port if self.mon.port else "NOT CONNECTED")
        self._update_cell(self.table, self.lora_row_map, "baud", str(self.mon.baud) if self.mon.baud else "-")
        self._update_cell(self.table, self.lora_row_map, "target", str(self.mari.address))
        self._update_cell(self.table, self.lora_row_map, "display", "ON" if self.mon.display else "OFF", color_hex="#55FF55" if self.mon.display else "#FF5555", bold=True)
        self._update_cell(self.table, self.lora_row_map, "data", self.mon.data.decode(errors='replace') if self.mon.data else "-")
        self._update_cell(self.table, self.lora_row_map, "len", str(len(self.mon.data) if self.mon.data else 0))        
        self._update_cell(self.table, self.lora_row_map, "sequence", parsed_sequence)
        self._update_cell(self.table, self.lora_row_map, "uuid", parsed_uuid)
        if self.mon.data:
            type_colors = {
                "U": ("USER", "#FFFFFF"), "I": ("IMU", "#FF55FF"), "P": ("PID", "#FFFF55"),
                "M": ("MOTOR", "#55FF55"), "E": ("ESC", "#5555FF"), "B": ("BATTERY", "#FF5555"),
                "RW": ("RAW", "#AAAAAA"), "MARI": ("MARI", "#55FFFF"),
                "C": ("CPU", "#FF55FF"), "R": ("TASK RATE", "#FFFF55"),
                "Q": ("Q-TUNE", "#FF55FF"), "F": ("F-TUNE", "#55FFFF")
            }
            type_label, type_color = type_colors.get(parsed_type, ("UNKNOWN" if parsed_type else "-", "#FF5555"))
            self._update_cell(self.table, self.lora_row_map, "type", type_label, color_hex=type_color, bold=True)
            
            if isinstance(parsed_payload, list):
                payload_str = ", ".join(str(v) for v in parsed_payload)
            else:
                payload_str = str(parsed_payload)
            self._update_cell(self.table, self.lora_row_map, "payload", payload_str)
        else:
            self._update_cell(self.table, self.lora_row_map, "type", "-")
            self._update_cell(self.table, self.lora_row_map, "payload", "-")

        stat = self.mari.sensor_stat
        
        self._update_cell(self.table_sensors, self.sensor_row_map, "pitch", f"{stat.pitch:.2f}")
        self._update_cell(self.table_sensors, self.sensor_row_map, "roll", f"{stat.roll:.2f}")
        self._update_cell(self.table_sensors, self.sensor_row_map, "yaw", f"{stat.yaw:.2f}")
        self._update_cell(self.table_sensors, self.sensor_row_map, "temp", f"{stat.temperature:.1f}")
        self._update_cell(self.table_sensors, self.sensor_row_map, "pressure", f"{stat.pressure:.1f}")
        self._update_cell(self.table_sensors, self.sensor_row_map, "alt", f"{stat.altitude:.2f}")
        self._update_cell(self.table_sensors, self.sensor_row_map, "batt_volt", f"{stat.battery_voltage:.2f}")
        self._update_cell(self.table_sensors, self.sensor_row_map, "batt_status", stat.battery_status.name,
                                                                                color_hex="#55FF55" if stat.battery_status == parsestuff.VoltageLevel.NORMAL
                                                                                else "#FFFF55" if stat.battery_status == parsestuff.VoltageLevel.WARNING
                                                                                else "#FF5555", bold=True)
        self._update_cell(self.table_sensors, self.sensor_row_map, "motorNW", f"{stat.motorNW:.2f}")
        self._update_cell(self.table_sensors, self.sensor_row_map, "motorNE", f"{stat.motorNE:.2f}")
        self._update_cell(self.table_sensors, self.sensor_row_map, "motorSW", f"{stat.motorSW:.2f}")
        self._update_cell(self.table_sensors, self.sensor_row_map, "motorSE", f"{stat.motorSE:.2f}")

        # dt (s) -> ms; max dt is the max of the last 100 samples
        dt_ms = stat.dt * 1000.0
        self._dt_window.append(dt_ms)
        max_dt_ms = max(self._dt_window) if self._dt_window else 0.0
        self._update_cell(self.table_sensors, self.sensor_row_map, "dt_ms", f"{dt_ms:.2f}")
        self._update_cell(self.table_sensors, self.sensor_row_map, "max_dt_ms", f"{max_dt_ms:.2f}")

        pitch_p = getattr(stat, 'pitchP', 0.0)
        pitch_i = getattr(stat, 'pitchI', 0.0)
        pitch_d = getattr(stat, 'pitchD', 0.0)
        roll_p = getattr(stat, 'rollP', 0.0)
        roll_i = getattr(stat, 'rollI', 0.0)
        roll_d = getattr(stat, 'rollD', 0.0)
        yaw_p = getattr(stat, 'yawP', 0.0)
        yaw_i = getattr(stat, 'yawI', 0.0)
        yaw_d = getattr(stat, 'yawD', 0.0)
        
        self._update_cell(self.table_pids, self.pids_row_map, "pitchP", f"{pitch_p:.2f}")
        self._update_cell(self.table_pids, self.pids_row_map, "pitchI", f"{pitch_i:.2f}")
        self._update_cell(self.table_pids, self.pids_row_map, "pitchD", f"{pitch_d:.2f}")
        self._update_cell(self.table_pids, self.pids_row_map, "rollP", f"{roll_p:.2f}")
        self._update_cell(self.table_pids, self.pids_row_map, "rollI", f"{roll_i:.2f}")
        self._update_cell(self.table_pids, self.pids_row_map, "rollD", f"{roll_d:.2f}")
        self._update_cell(self.table_pids, self.pids_row_map, "yawP", f"{yaw_p:.2f}")
        self._update_cell(self.table_pids, self.pids_row_map, "yawI", f"{yaw_i:.2f}")
        self._update_cell(self.table_pids, self.pids_row_map, "yawD", f"{yaw_d:.2f}")
        self._update_cell(self.table_pids, self.pids_row_map, "pitchPID", f"{stat.pitchPID:.2f}")
        self._update_cell(self.table_pids, self.pids_row_map, "rollPID", f"{stat.rollPID:.2f}")
        self._update_cell(self.table_pids, self.pids_row_map, "yawPID", f"{stat.yawPID:.2f}")

        # --- RTOS / CPU TABLE ---
        rtos = self.mari.rtos_stat
        self._update_cell(self.table_cpu, self.cpu_row_map, "flight_rate", str(rtos.flight_rate))
        self._update_cell(self.table_cpu, self.cpu_row_map, "tele_tx_rate", str(rtos.tele_tx_rate))
        self._update_cell(self.table_cpu, self.cpu_row_map, "tele_rx_rate", str(rtos.tele_rx_rate))
        self._update_cell(self.table_cpu, self.cpu_row_map, "housekeeping_rate", str(rtos.housekeeping_rate))
        self._update_cell(self.table_cpu, self.cpu_row_map, "flight_cpu_per", f"{rtos.flight_cpu_per:.1f}")
        self._update_cell(self.table_cpu, self.cpu_row_map, "tele_tx_cpu_per", f"{rtos.tele_tx_cpu_per:.1f}")
        self._update_cell(self.table_cpu, self.cpu_row_map, "tele_rx_cpu_per", f"{rtos.tele_rx_cpu_per:.1f}")
        self._update_cell(self.table_cpu, self.cpu_row_map, "housekeeping_cpu_per", f"{rtos.housekeeping_cpu_per:.1f}")
        self._update_cell(self.table_cpu, self.cpu_row_map, "idle_cpu_per", f"{rtos.idle_cpu_per:.1f}")

        # --- UPDATE GRAPHS ---
        current_time = time.time() - self.plot_start_time
        
        self.plot_time.append(current_time)
        
        self.plot_rssi.append(self.mari.rssi if self.mari.rssi else 0)
        self.plot_snr.append(self.mari.snr if self.mari.snr else 0)
        
        self.plot_pitch.append(stat.pitch)
        self.plot_roll.append(stat.roll)
        self.plot_yaw.append(stat.yaw)
        self.plot_pid_pitch.append(stat.pitchPID)
        self.plot_pid_roll.append(stat.rollPID)
        self.plot_pid_yaw.append(stat.yawPID)
        self.plot_mNW.append(stat.motorNW)
        self.plot_mNE.append(stat.motorNE)
        self.plot_mSW.append(stat.motorSW)
        self.plot_mSE.append(stat.motorSE)
        
        self.plot_pitchP.append(pitch_p)
        self.plot_pitchI.append(pitch_i)
        self.plot_pitchD.append(pitch_d)
        self.plot_rollP.append(roll_p)
        self.plot_rollI.append(roll_i)
        self.plot_rollD.append(roll_d)
        self.plot_yawP.append(yaw_p)
        self.plot_yawI.append(yaw_i)
        self.plot_yawD.append(yaw_d)

        # Fine-tuning-specific buffers (Q + F frames). Attitude/motors reuse
        # the existing plot_pitch/roll/yaw and plot_mNW/NE/SW/SE buffers.
        self.plot_pitch_user.append(getattr(stat, 'pitch_input', 0.0))
        self.plot_roll_user.append(getattr(stat, 'roll_input', 0.0))
        self.plot_yaw_user.append(getattr(stat, 'yaw_input', 0.0))
        self.plot_pitchError.append(getattr(stat, 'pitchError', 0.0))
        self.plot_rollError.append(getattr(stat, 'rollError', 0.0))
        self.plot_yawError.append(getattr(stat, 'yawError', 0.0))

        t_data = list(self.plot_time)
        
        self.line_rssi.setData(t_data, list(self.plot_rssi))
        self.line_snr.setData(t_data, list(self.plot_snr))
        
        self.line_pitch.setData(t_data, list(self.plot_pitch))
        self.line_roll.setData(t_data, list(self.plot_roll))
        self.line_yaw.setData(t_data, list(self.plot_yaw))
        self.line_pid_pitch.setData(t_data, list(self.plot_pid_pitch))
        self.line_pid_roll.setData(t_data, list(self.plot_pid_roll))
        self.line_pid_yaw.setData(t_data, list(self.plot_pid_yaw))
        self.line_mNW.setData(t_data, list(self.plot_mNW))
        self.line_mNE.setData(t_data, list(self.plot_mNE))
        self.line_mSW.setData(t_data, list(self.plot_mSW))
        self.line_mSE.setData(t_data, list(self.plot_mSE))
        
        self.line_pitchP.setData(t_data, list(self.plot_pitchP))
        self.line_pitchI.setData(t_data, list(self.plot_pitchI))
        self.line_pitchD.setData(t_data, list(self.plot_pitchD))
        self.line_rollP.setData(t_data, list(self.plot_rollP))
        self.line_rollI.setData(t_data, list(self.plot_rollI))
        self.line_rollD.setData(t_data, list(self.plot_rollD))
        self.line_yawP.setData(t_data, list(self.plot_yawP))
        self.line_yawI.setData(t_data, list(self.plot_yawI))
        self.line_yawD.setData(t_data, list(self.plot_yawD))

        # Fine-tuning panes
        self.line_ft_pitch.setData(t_data, list(self.plot_pitch))
        self.line_ft_roll.setData(t_data, list(self.plot_roll))
        self.line_ft_yaw.setData(t_data, list(self.plot_yaw))
        self.line_ft_pitch_user.setData(t_data, list(self.plot_pitch_user))
        self.line_ft_roll_user.setData(t_data, list(self.plot_roll_user))
        self.line_ft_yaw_user.setData(t_data, list(self.plot_yaw_user))
        self.line_ft_pitchE.setData(t_data, list(self.plot_pitchError))
        self.line_ft_rollE.setData(t_data, list(self.plot_rollError))
        self.line_ft_yawE.setData(t_data, list(self.plot_yawError))
        self.line_ft_pitchI.setData(t_data, list(self.plot_pitchI))
        self.line_ft_rollI.setData(t_data, list(self.plot_rollI))
        self.line_ft_yawI.setData(t_data, list(self.plot_yawI))
        self.line_ft_pitchD.setData(t_data, list(self.plot_pitchD))
        self.line_ft_rollD.setData(t_data, list(self.plot_rollD))
        self.line_ft_yawD.setData(t_data, list(self.plot_yawD))
        self.line_ft_m0.setData(t_data, list(self.plot_mNW))
        self.line_ft_m1.setData(t_data, list(self.plot_mNE))
        self.line_ft_m2.setData(t_data, list(self.plot_mSW))
        self.line_ft_m3.setData(t_data, list(self.plot_mSE))

    def _update_stopwatch_ui(self):
        if self.timer_running:
            elapsed = time.perf_counter() - self.start_time + self.elapsed_before_stop
            minutes, seconds = int(elapsed // 60), int(elapsed % 60)
            tenths = int((elapsed - int(elapsed)) * 10)
            self._update_cell(self.table_sensors, self.sensor_row_map, "time", f"{minutes:02d}:{seconds:02d}.{tenths}")

    def closeEvent(self, event):
        try:
            self.mon.close()
            self.mon.close_log()
        except Exception:
            pass
        event.accept()