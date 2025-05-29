import sys
import json
import threading
import numpy as np
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QVBoxLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import serial  # Make sure pyserial is installed
from PyQt5.QtCore import pyqtSignal
from blockchain import append_block, verify_chain
from PyQt5.QtWidgets import QGraphicsScene
import os
from PyQt5.QtGui import QPen
from PyQt5.QtCore import Qt

import re

ANSI_ESCAPE = re.compile(r'\x1B[@-_][0-?]*[ -/]*[@-~]')

# --- Radar Plot ---
class TrackingRadarCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(4, 3), dpi=100)
        self.ax = self.fig.add_subplot(111, polar=True)
        self.fig.patch.set_facecolor("#1e1e1e")  # Full figure background
        self.ax.set_facecolor("#1e1e1e")         # Polar plot area
        self.ax.tick_params(colors='white')  # Radial + angular tick marks
        self.ax.spines['polar'].set_color('white')  # Outer circle
        super().__init__(self.fig)
        self.setParent(parent)

        # Polar plot config
        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(-1)
        self.ax.set_thetamin(0)
        self.ax.set_thetamax(180)

        # Set radial limit to 350 cm
        self.ax.set_ylim(0, 350)

        # Optional: move radial labels to top-center
        self.ax.set_rlabel_position(90)

        # Manually set clean radial ticks with units
        r_ticks = [50, 100, 150, 200, 250, 300, 350]
        self.ax.set_yticks(r_ticks)
        self.ax.set_yticklabels([f"{r} cm" for r in r_ticks], color='white')

        # Plot elements
        self.sweep_line, = self.ax.plot([], [], 'g-')
        self.dot, = self.ax.plot([], [], 'ro')

    def update_sweep(self, angle_deg):
        theta = np.radians(angle_deg)
        self.sweep_line.set_data([0, theta], [0, 350])
        self.dot.set_data([], [])
        self.draw()

    def update_tracking(self, angle_deg, distance):
        theta = np.radians(angle_deg)
        self.sweep_line.set_data([], [])
        self.dot.set_data([theta], [distance])
        self.draw()



# --- Main App Window ---
class MainWindow(QtWidgets.QMainWindow):
    uart_log = pyqtSignal(str)
    def __init__(self):
        super(MainWindow, self).__init__()
        uic.loadUi("template.ui", self)

        self.uart_log.connect(self.consoleBox.appendPlainText)

        self.blockchain_scene = QGraphicsScene()
        self.blockchain.setScene(self.blockchain_scene)

        self.visButton.clicked.connect(self.visualize_blockchain)
        self.verifyButton.clicked.connect(self.verify_blockchain)

        self.verifyLabel

        self.canvas = TrackingRadarCanvas(self.tracking)
        layout = QVBoxLayout(self.tracking)
        layout.addWidget(self.canvas)

        self.sendButton.clicked.connect(self.send_command)

        self.serial_port = serial.Serial('COM8', 115200, timeout=1)  # Change COM port as needed
        self.running = True
        # self.sweep_angle = 0

        threading.Thread(target=self.read_uart, daemon=True).start()

    def visualize_blockchain(self):
        self.blockchain_scene.clear()

        if not os.path.exists("blockchain.json"):
            self.consoleBox.appendPlainText("⚠️ No blockchain data found.")
            return

        with open("blockchain.json", "r") as f:
            chain = json.load(f)

        block_width = 150
        block_height = 80
        spacing = 30
        x = 0
        y = 0

        for i, block in enumerate(chain):
            # Use a filled rectangle with light background
            rect = self.blockchain_scene.addRect(x, y, block_width, block_height, pen=QPen(Qt.white, 2))
            rect.setBrush(Qt.darkCyan)  # Color fill for block box

            # Add text with white color
            index = block['index']
            short_hash = block['hash'][:10]
            timestamp = block['timestamp'].split("T")[1][:8]  # Just time (HH:MM:SS)

            text_item = self.blockchain_scene.addText(f"#{index}\n{short_hash}...\n{timestamp}")
            text_item.setDefaultTextColor(Qt.white)
            text_item.setPos(x + 10, y + 10)

            # Draw arrow line to next block
            if i < len(chain) - 1:
                self.blockchain_scene.addLine(
                    x + block_width, y + block_height / 2,
                    x + block_width + spacing, y + block_height / 2,
                    pen=QPen(Qt.white, 2)
                )

            x += block_width + spacing

        self.consoleBox.appendPlainText(f"Visualized {len(chain)} blocks.")


    def verify_blockchain(self):
        if not os.path.exists("blockchain.json"):
            self.consoleBox.appendPlainText("⚠️ No blockchain data to verify.")
            return

        is_valid = verify_chain()
        if is_valid:
            self.consoleBox.appendPlainText("✅ Blockchain is valid.")
            self.verifyLabel.setText("✅ Blockchain is valid.")
        else:
            self.consoleBox.appendPlainText("❌ Blockchain is broken!")
            self.verifyLabel.setText("❌ Blockchain is broken!")

    def send_command(self):
        msg = self.sendBox.toPlainText().strip()
        if msg:
            self.serial_port.write((msg + '\n').encode())
            self.consoleBox.appendPlainText(f"> {msg}")

    def read_uart(self):
        while self.running:
            try:
                raw = self.serial_port.readline()
                if not raw:
                    print("[DEBUG] No data read (timeout or disconnect?)")
                    continue

                print(f"[RAW BYTES] {raw}")
                line = raw.decode(errors='ignore').strip()

                # Clean garbage and extract JSON only
                idx_start = line.find("{")
                idx_end = line.rfind("}")
                if idx_start == -1 or idx_end == -1 or idx_end < idx_start:
                    print(f"[SKIP] Invalid structure: {line}")
                    continue

                line = line[idx_start:idx_end + 1]

                try:
                    data = json.loads(line)
                except json.JSONDecodeError as e:
                    print(f"[JSON ERROR] {e} → {line}")
                    continue
                    
                # Parse and use
                angle = float(data.get("angle", 0))
                distance = float(data.get("distance", 0))
                tracking = bool(data.get("tracking", 0))

                try:
                    append_block({
                        "angle": angle,
                        "distance": distance,
                        "tracking": tracking
                    })
                except Exception as e:
                    print(f"[BLOCKCHAIN ERROR] {type(e).__name__}: {e}")

                print(f"[PARSED] Angle: {angle}, Distance: {distance}, Tracking: {tracking}")

                self.uart_log.emit(f"< {line}")
                self.angleLabel.setText(f"Angle: {angle:.1f}°")
                self.distanceLabel.setText(f"Distance: {distance:.1f} cm")
                self.trackingLabel.setText(f"Tracking: {'YES' if tracking else 'NO'}")

                if tracking:
                    self.canvas.update_tracking(angle, distance)
                else:
                    self.canvas.update_sweep(angle)

            except Exception as e:
                print(f"[CRASH] {type(e).__name__}: {e}")
                self.consoleBox.appendPlainText(f"[Error] {type(e).__name__}: {e}")


    def closeEvent(self, event):
        self.running = False
        if self.serial_port.is_open:
            self.serial_port.close()
        event.accept()

# --- App Entry ---
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet("""
        QWidget {
            background-color: #1e1e1e;
            color: #ffffff;
            font-family: 'Segoe UI', sans-serif;
            font-size: 12pt;
        }

        QPlainTextEdit, QTextEdit, QLineEdit {
            background-color: #2e2e2e;
            border: 1px solid #555;
            padding: 4px;
        }

        QPushButton {
            background-color: #3c3c3c;
            border: 1px solid #888;
            padding: 6px;
            border-radius: 4px;
        }

        QPushButton:hover {
            background-color: #505050;
        }

        QLabel {
            font-weight: bold;
        }

        QGraphicsView {
            background-color: #111;
            border: 1px solid #555;
        }
    """)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
