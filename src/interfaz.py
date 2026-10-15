"""Live acquisition GUI for the STM32 ADS1115 data logger.

Run from the project root with: python src/interfaz.py
The serial protocol accepts DATA records from the current firmware and the
legacy human-readable ``T:... | t:...`` format.
"""

from __future__ import annotations

import csv
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import serial
from serial.tools import list_ports
from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg


SENSOR_NAMES = ("Temperature 1", "Temperature 2", "Temperature 3",
                "Temperature 4", "Voltage", "Current")


@dataclass(frozen=True)
class Sample:
    elapsed_ms: int
    values: tuple[float, ...]


def parse_line(line: str) -> Optional[Sample]:
    """Parse both DATA CSV records and the original compact serial format."""
    line = line.strip()
    if line.startswith("DATA,"):
        try:
            fields = line.split(",")
            if len(fields) != 8:
                return None
            return Sample(int(fields[1]), tuple(float(value) for value in fields[2:]))
        except ValueError:
            return None

    match = re.match(
        r"T:\s*([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s*\|\s*t:(\d+)\s+V:([\d.-]+)\s+I:([\d.-]+)",
        line,
    )
    if not match:
        return None
    values = tuple(float(value) for value in match.groups()[:4])
    return Sample(int(match.group(5)), values + (float(match.group(6)), float(match.group(7))))


class AcquisitionStore(QtCore.QObject):
    sample_added = QtCore.Signal(object)

    def __init__(self) -> None:
        super().__init__()
        self.samples: list[Sample] = []

    def clear(self) -> None:
        self.samples.clear()

    def add(self, sample: Sample) -> None:
        self.samples.append(sample)
        self.sample_added.emit(sample)

    def export_csv(self, filename: str) -> None:
        with open(filename, "w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(["elapsed_ms", *SENSOR_NAMES])
            writer.writerows([[sample.elapsed_ms, *sample.values] for sample in self.samples])


class SerialWorker(QtCore.QThread):
    sample_received = QtCore.Signal(object)
    status_message = QtCore.Signal(str)
    connection_changed = QtCore.Signal(bool)

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        super().__init__()
        self.port_name = port
        self.baudrate = baudrate
        self._serial: Optional[serial.Serial] = None
        self._running = True

    def run(self) -> None:
        try:
            self._serial = serial.Serial(self.port_name, self.baudrate, timeout=0.2)
            self.connection_changed.emit(True)
            self.status_message.emit(f"Connected to {self.port_name}")
            while self._running:
                raw = self._serial.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    sample = parse_line(line)
                    if sample is not None:
                        self.sample_received.emit(sample)
                    elif line:
                        self.status_message.emit(line)
        except (serial.SerialException, OSError) as error:
            self.status_message.emit(f"Serial error: {error}")
        finally:
            self.connection_changed.emit(False)
            if self._serial is not None:
                self._serial.close()

    def send(self, command: str) -> None:
        if self._serial is not None and self._serial.is_open:
            self._serial.write((command.strip() + "\n").encode("ascii"))

    def stop(self) -> None:
        self._running = False
        self.wait(1500)


class PlotPanel(pg.PlotWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setLabel("bottom", "Time", units="s")
        self.showGrid(x=True, y=True, alpha=0.25)
        self.addLegend()
        self.curves = [
            self.plot(pen=pg.intColor(index, hues=6), name=name)
            for index, name in enumerate(SENSOR_NAMES)
        ]

    def refresh(self, samples: list[Sample], visible: list[bool]) -> None:
        time_values = [sample.elapsed_ms / 1000 for sample in samples]
        for index, curve in enumerate(self.curves):
            curve.setVisible(visible[index])
            curve.setData(time_values, [sample.values[index] for sample in samples])


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ADS1115 Data Acquisition")
        self.resize(1100, 700)
        self.store = AcquisitionStore()
        self.worker: Optional[SerialWorker] = None
        self.started_at: Optional[float] = None
        self._build_ui()
        self._refresh_ports()
        self.store.sample_added.connect(self._sample_added)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        connection = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_button = QtWidgets.QPushButton("Refresh ports")
        self.connect_button = QtWidgets.QPushButton("Connect")
        connection.addWidget(QtWidgets.QLabel("Serial port:"))
        connection.addWidget(self.port_combo, 1)
        connection.addWidget(self.refresh_button)
        connection.addWidget(self.connect_button)
        root.addLayout(connection)

        controls = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("Start measurement")
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.clear_button = QtWidgets.QPushButton("Clear")
        self.export_button = QtWidgets.QPushButton("Export CSV")
        self.elapsed_label = QtWidgets.QLabel("Elapsed: 00:00:00.000")
        for button in (self.start_button, self.stop_button, self.clear_button, self.export_button):
            controls.addWidget(button)
        controls.addStretch()
        controls.addWidget(self.elapsed_label)
        root.addLayout(controls)

        content = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.plot = PlotPanel()
        content.addWidget(self.plot)
        sensor_panel = QtWidgets.QGroupBox("Sensors")
        sensor_layout = QtWidgets.QVBoxLayout(sensor_panel)
        self.sensor_checks: list[QtWidgets.QCheckBox] = []
        for name in SENSOR_NAMES:
            check = QtWidgets.QCheckBox(name)
            check.setChecked(True)
            check.toggled.connect(self._refresh_plot)
            self.sensor_checks.append(check)
            sensor_layout.addWidget(check)
        sensor_layout.addStretch()
        content.addWidget(sensor_panel)
        content.setStretchFactor(0, 1)
        root.addWidget(content, 1)

        self.table = QtWidgets.QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["Time (s)", *SENSOR_NAMES])
        self.table.horizontalHeader().setStretchLastSection(True)
        root.addWidget(self.table)
        self.statusBar().showMessage("Disconnected")

        self.refresh_button.clicked.connect(self._refresh_ports)
        self.connect_button.clicked.connect(self._toggle_connection)
        self.start_button.clicked.connect(self._start_measurement)
        self.stop_button.clicked.connect(self._stop_measurement)
        self.clear_button.clicked.connect(self._clear)
        self.export_button.clicked.connect(self._export)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._update_elapsed)

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        self.port_combo.addItems([port.device for port in list_ports.comports()])
        if current:
            index = self.port_combo.findText(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def _toggle_connection(self) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
            self.connect_button.setText("Connect")
            return
        if not self.port_combo.currentText():
            self.statusBar().showMessage("Select a serial port first")
            return
        self.worker = SerialWorker(self.port_combo.currentText())
        self.worker.sample_received.connect(self.store.add)
        self.worker.status_message.connect(self.statusBar().showMessage)
        self.worker.connection_changed.connect(
            lambda connected: self.connect_button.setText("Disconnect" if connected else "Connect")
        )
        self.worker.start()

    def _send(self, command: str) -> None:
        if self.worker is None:
            self.statusBar().showMessage("Connect to the board first")
        else:
            self.worker.send(command)

    def _start_measurement(self) -> None:
        self.store.clear()
        self.table.setRowCount(0)
        self.started_at = time.monotonic()
        self.timer.start(50)
        self._send("START")
        self.statusBar().showMessage("Measurement started")

    def _stop_measurement(self) -> None:
        self._send("STOP")
        self.timer.stop()
        self.started_at = None
        self.statusBar().showMessage("Measurement stopped")

    def _clear(self) -> None:
        self.store.clear()
        self.table.setRowCount(0)
        self.plot.refresh([], [check.isChecked() for check in self.sensor_checks])

    def _sample_added(self, sample: Sample) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        values = [sample.elapsed_ms / 1000, *sample.values]
        for column, value in enumerate(values):
            self.table.setItem(row, column, QtWidgets.QTableWidgetItem(f"{value:.3f}"))
        self.table.scrollToBottom()
        self._refresh_plot()

    def _refresh_plot(self) -> None:
        self.plot.refresh(self.store.samples, [check.isChecked() for check in self.sensor_checks])

    def _update_elapsed(self) -> None:
        if self.started_at is not None:
            elapsed = time.monotonic() - self.started_at
            self.elapsed_label.setText(f"Elapsed: {elapsed:09.3f} s")

    def _export(self) -> None:
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export acquisition", "acquisition.csv", "CSV files (*.csv)"
        )
        if filename:
            try:
                self.store.export_csv(filename)
                self.statusBar().showMessage(f"Exported {len(self.store.samples)} samples")
            except OSError as error:
                QtWidgets.QMessageBox.critical(self, "Export failed", str(error))

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self.worker is not None:
            self.worker.stop()
        event.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("ADS1115 Data Acquisition")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
