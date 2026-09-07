
"""
AeroNotts Live Ground-Station Data Display (Complete PC Program)
------------------------------------------
Shows each validated telemetry packet as one new row.
No plotting.

The LoRa recovery system remains inside ground_station.ino.
Recovered packets are displayed and marked, but do not disturb radio handling.

Install:
    pip install pyserial pyside6

Run:
    python ground_live_display.py
"""

import csv
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import serial
import serial.tools.list_ports
from PySide6 import QtCore, QtWidgets

BAUD = 115200
SERIAL_TIMEOUT_S = 0.25
RECONNECT_DELAY_S = 1.5

# Keep only this many rows visibly in the GUI.
# ALL unique packets are still written to CSV.
MAX_VISIBLE_ROWS = 1000

COLUMNS = [
    "Packet",
    "Time (s)",
    "Acc X",
    "Acc Y",
    "Acc Z",
    "Gyro X",
    "Gyro Y",
    "Gyro Z",
    "Latitude",
    "Longitude",
    "Pressure",
    "Temp",
    "Altitude",
    "Status",
    "RSSI",
    "SNR",
    "Recovered",
]

CSV_COLUMNS = [
    "Team",
    "Packet",
    "Time_ms",
    "Ax_mps2",
    "Ay_mps2",
    "Az_mps2",
    "Gx_dps",
    "Gy_dps",
    "Gz_dps",
    "Latitude",
    "Longitude",
    "Pressure_Pa",
    "Temperature_C",
    "Altitude_m",
    "Status",
    "RSSI_dBm",
    "SNR_dB",
    "Recovered",
    "PC_UTC",
]


def parse_data_line(line: str):
    parts = line.strip().split(",")
    if len(parts) != 19 or parts[0] != "D":
        return None

    try:
        return {
            "Team": parts[1],
            "Packet": int(parts[2]),
            "Time_ms": int(parts[3]),
            "Ax_mps2": float(parts[4]),
            "Ay_mps2": float(parts[5]),
            "Az_mps2": float(parts[6]),
            "Gx_dps": float(parts[7]),
            "Gy_dps": float(parts[8]),
            "Gz_dps": float(parts[9]),
            "Latitude": float(parts[10]),
            "Longitude": float(parts[11]),
            "Pressure_Pa": int(parts[12]),
            "Temperature_C": float(parts[13]),
            "Altitude_m": float(parts[14]),
            "Status": int(parts[15]),
            "RSSI_dBm": float(parts[16]),
            "SNR_dB": float(parts[17]),
            "Recovered": int(parts[18]),
            "PC_UTC": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        }
    except ValueError:
        return None


class SerialWorker(QtCore.QThread):
    telemetry = QtCore.Signal(dict)
    diagnostic = QtCore.Signal(str)
    connection_state = QtCore.Signal(bool, str)

    def __init__(self, port: str):
        super().__init__()
        self.port = port
        self.running = True
        self.ser: Optional[serial.Serial] = None

    def stop(self):
        self.running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def run(self):
        while self.running:
            try:
                self.connection_state.emit(False, f"Connecting to {self.port}...")

                self.ser = serial.Serial(
                    self.port,
                    BAUD,
                    timeout=SERIAL_TIMEOUT_S,
                    write_timeout=1.0,
                )
                self.ser.reset_input_buffer()

                self.connection_state.emit(
                    True, f"Connected: {self.port} @ {BAUD}"
                )

                while self.running and self.ser.is_open:
                    raw = self.ser.readline()

                    if not raw:
                        continue

                    line = raw.decode("utf-8", errors="replace").strip()

                    if not line:
                        continue

                    if line.startswith("#"):
                        self.diagnostic.emit(line)
                        continue

                    row = parse_data_line(line)

                    if row is not None:
                        self.telemetry.emit(row)

            except (serial.SerialException, OSError) as exc:
                self.connection_state.emit(
                    False, f"Disconnected: {exc}"
                )

                if self.running:
                    time.sleep(RECONNECT_DELAY_S)

            finally:
                try:
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass


class CsvLogger:
    def __init__(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.arrival_path = Path(
            f"AeroNotts_live_{stamp}.csv"
        )

        self.sorted_path = Path(
            f"AeroNotts_sorted_{stamp}.csv"
        )

        self.file = self.arrival_path.open(
            "w",
            newline="",
            buffering=1
        )

        self.writer = csv.DictWriter(
            self.file,
            fieldnames=CSV_COLUMNS
        )

        self.writer.writeheader()

    def append(self, row):
        self.writer.writerow(row)

    def write_sorted(self, packet_store):
        temp = self.sorted_path.with_suffix(".tmp")

        with temp.open("w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=CSV_COLUMNS
            )

            writer.writeheader()

            for packet_no in sorted(packet_store):
                writer.writerow(packet_store[packet_no])

        temp.replace(self.sorted_path)

    def close(self):
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        self.setWindowTitle(
            "AeroNotts CanSat Ground Station"
        )

        self.resize(1500, 700)

        self.worker = None
        self.logger = None

        self.packet_store = {}
        self.highest_packet = None

        self.received_count = 0
        self.recovered_count = 0
        self.duplicate_count = 0

        self.build_ui()
        self.refresh_ports()

        self.sorted_timer = QtCore.QTimer(self)
        self.sorted_timer.timeout.connect(
            self.write_sorted_csv
        )
        self.sorted_timer.start(5000)

    def build_ui(self):

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        layout = QtWidgets.QVBoxLayout(central)

        # ------------------------------------------------
        # Connection controls
        # ------------------------------------------------
        top = QtWidgets.QHBoxLayout()

        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(160)

        self.refresh_button = QtWidgets.QPushButton(
            "Refresh Ports"
        )

        self.connect_button = QtWidgets.QPushButton(
            "Connect"
        )

        self.connection_label = QtWidgets.QLabel(
            "Disconnected"
        )

        self.refresh_button.clicked.connect(
            self.refresh_ports
        )

        self.connect_button.clicked.connect(
            self.toggle_connection
        )

        top.addWidget(QtWidgets.QLabel("ESP32 Port:"))
        top.addWidget(self.port_combo)
        top.addWidget(self.refresh_button)
        top.addWidget(self.connect_button)
        top.addWidget(self.connection_label, 1)

        layout.addLayout(top)

        # ------------------------------------------------
        # Current link information
        # ------------------------------------------------
        summary = QtWidgets.QHBoxLayout()

        self.current_packet_label = QtWidgets.QLabel(
            "Latest packet: --"
        )

        self.received_label = QtWidgets.QLabel(
            "Received: 0"
        )

        self.missing_label = QtWidgets.QLabel(
            "Missing: 0"
        )

        self.recovered_label = QtWidgets.QLabel(
            "Recovered: 0"
        )

        summary.addWidget(self.current_packet_label)
        summary.addWidget(self.received_label)
        summary.addWidget(self.missing_label)
        summary.addWidget(self.recovered_label)
        summary.addStretch()

        layout.addLayout(summary)

        # ------------------------------------------------
        # Live telemetry table
        # ------------------------------------------------
        self.table = QtWidgets.QTableWidget()

        self.table.setColumnCount(len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)

        self.table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )

        self.table.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectRows
        )

        self.table.setAlternatingRowColors(True)

        self.table.verticalHeader().setVisible(False)

        self.table.horizontalHeader().setStretchLastSection(
            True
        )

        self.table.setSortingEnabled(False)

        layout.addWidget(self.table, 1)

        # ------------------------------------------------
        # Diagnostics
        # ------------------------------------------------
        self.diagnostics = QtWidgets.QPlainTextEdit()
        self.diagnostics.setReadOnly(True)
        self.diagnostics.setMaximumHeight(100)
        self.diagnostics.document().setMaximumBlockCount(100)

        layout.addWidget(self.diagnostics)

        self.csv_label = QtWidgets.QLabel(
            "CSV: not started"
        )

        self.statusBar().addPermanentWidget(
            self.csv_label
        )

    def refresh_ports(self):

        current = self.port_combo.currentText()

        self.port_combo.clear()

        ports = [
            p.device
            for p in serial.tools.list_ports.comports()
        ]

        self.port_combo.addItems(ports)

        if current in ports:
            self.port_combo.setCurrentText(current)

    def toggle_connection(self):

        if self.worker is not None:
            self.disconnect_serial()
            return

        port = self.port_combo.currentText().strip()

        if not port:
            self.connection_label.setText(
                "Select a COM port"
            )
            return

        self.packet_store.clear()

        self.highest_packet = None
        self.received_count = 0
        self.recovered_count = 0
        self.duplicate_count = 0

        self.table.setRowCount(0)

        self.logger = CsvLogger()

        self.csv_label.setText(
            f"CSV: {self.logger.arrival_path.name}"
        )

        self.worker = SerialWorker(port)

        self.worker.telemetry.connect(
            self.on_telemetry
        )

        self.worker.diagnostic.connect(
            self.on_diagnostic
        )

        self.worker.connection_state.connect(
            self.on_connection_state
        )

        self.worker.start()

        self.connect_button.setText(
            "Disconnect"
        )

        self.port_combo.setEnabled(False)
        self.refresh_button.setEnabled(False)

    def disconnect_serial(self):

        if self.worker is not None:

            self.worker.stop()
            self.worker.wait(2500)

            self.worker = None

        if self.logger is not None:

            self.logger.write_sorted(
                self.packet_store
            )

            self.logger.close()

            self.logger = None

        self.connection_label.setText(
            "Disconnected"
        )

        self.connect_button.setText(
            "Connect"
        )

        self.port_combo.setEnabled(True)
        self.refresh_button.setEnabled(True)

    @QtCore.Slot(dict)
    def on_telemetry(self, row):

        packet = row["Packet"]

        # Duplicate retransmission - ignore.
        if packet in self.packet_store:

            self.duplicate_count += 1
            return

        self.packet_store[packet] = row

        self.received_count += 1

        if row["Recovered"]:
            self.recovered_count += 1

        if (
            self.highest_packet is None
            or packet > self.highest_packet
        ):
            self.highest_packet = packet

        if self.logger is not None:
            self.logger.append(row)

        self.add_table_row(row)
        self.update_summary()

    def add_table_row(self, row):

        # Remove oldest visible row if GUI table is full.
        # CSV remains complete.
        if self.table.rowCount() >= MAX_VISIBLE_ROWS:
            self.table.removeRow(0)

        r = self.table.rowCount()
        self.table.insertRow(r)

        values = [
            str(row["Packet"]),
            f'{row["Time_ms"] / 1000.0:.3f}',
            f'{row["Ax_mps2"]:.3f}',
            f'{row["Ay_mps2"]:.3f}',
            f'{row["Az_mps2"]:.3f}',
            f'{row["Gx_dps"]:.1f}',
            f'{row["Gy_dps"]:.1f}',
            f'{row["Gz_dps"]:.1f}',
            f'{row["Latitude"]:.7f}',
            f'{row["Longitude"]:.7f}',
            str(row["Pressure_Pa"]),
            f'{row["Temperature_C"]:.2f}',
            f'{row["Altitude_m"]:.3f}',
            f'0x{row["Status"]:02X}',
            f'{row["RSSI_dBm"]:.1f}',
            f'{row["SNR_dB"]:.1f}',
            "YES" if row["Recovered"] else "",
        ]

        for col, value in enumerate(values):
            item = QtWidgets.QTableWidgetItem(value)

            if row["Recovered"]:
                font = item.font()
                font.setItalic(True)
                item.setFont(font)

            self.table.setItem(r, col, item)

        # Always show newest received row.
        self.table.scrollToBottom()

    def update_summary(self):

        missing = 0

        if (
            self.packet_store
            and self.highest_packet is not None
        ):

            # Packet numbering starts at 0 for a freshly erased mission.
            # This also shows missing packets from the beginning of the flight.
            expected = self.highest_packet + 1

            present = sum(
                1
                for p in self.packet_store
                if 0 <= p <= self.highest_packet
            )

            missing = max(
                0,
                expected - present
            )

        self.current_packet_label.setText(
            f"Latest packet: "
            f"{self.highest_packet if self.highest_packet is not None else '--'}"
        )

        self.received_label.setText(
            f"Received: {self.received_count}"
        )

        self.missing_label.setText(
            f"Missing: {missing}"
        )

        self.recovered_label.setText(
            f"Recovered: {self.recovered_count}"
        )

    @QtCore.Slot(str)
    def on_diagnostic(self, line):

        self.diagnostics.appendPlainText(line)

    @QtCore.Slot(bool, str)
    def on_connection_state(self, connected, message):

        self.connection_label.setText(message)

    def write_sorted_csv(self):

        if (
            self.logger is not None
            and self.packet_store
        ):

            self.logger.write_sorted(
                self.packet_store
            )

    def closeEvent(self, event):

        self.disconnect_serial()

        event.accept()


def main():

    app = QtWidgets.QApplication(sys.argv)

    window = MainWindow()

    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()