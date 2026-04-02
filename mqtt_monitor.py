import sys
import json
import random
import string
from datetime import datetime

import paho.mqtt.client as mqtt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSpinBox, QDoubleSpinBox, QSlider, QTextEdit,
    QGroupBox, QGridLayout, QComboBox, QLineEdit
)
from PyQt5.QtCore import QTimer, Qt

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------
BROKER = "mqtt.hbo-ict.net"
PORT = 1883
BASE_TOPIC = "student/AD25VBa4"
CLIENT_ID = "dashboard_" + "".join(random.sample(string.ascii_letters, 8))

# All topics
TOPIC_CMD         = f"{BASE_TOPIC}/cmd"
TOPIC_STATUS      = f"{BASE_TOPIC}/robot_status"
TOPIC_PID_KP      = f"{BASE_TOPIC}/pid/kp"
TOPIC_PID_KI      = f"{BASE_TOPIC}/pid/ki"
TOPIC_PID_KD      = f"{BASE_TOPIC}/pid/kd"
TOPIC_SPEED       = f"{BASE_TOPIC}/speed"
TOPIC_PID_STATUS  = f"{BASE_TOPIC}/pid_status"
TOPIC_SPEED_STATUS= f"{BASE_TOPIC}/speed_status"
TOPIC_ROUTE       = f"{BASE_TOPIC}/route"
TOPIC_COMMAND     = f"{BASE_TOPIC}/command"
TOPIC_NODE        = f"{BASE_TOPIC}/node"
TOPIC_DEBUG       = f"{BASE_TOPIC}/debug"

# All topics we subscribe to (incoming)
SUBSCRIBE_TOPICS = [
    TOPIC_STATUS, TOPIC_PID_STATUS, TOPIC_SPEED_STATUS,
    TOPIC_NODE, TOPIC_DEBUG
]

# ------------------------------------------------------------
# PyQt5 Dashboard
# ------------------------------------------------------------
class RobotDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Lake Side Mania Robot Dashboard")
        self.setMinimumSize(800, 600)

        # MQTT client
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, CLIENT_ID)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_disconnect = self.on_disconnect

        # UI elements
        self.init_ui()

        # Connect to broker
        self.mqtt_client.connect(BROKER, PORT)
        self.mqtt_client.loop_start()

        # Timer to keep connection alive (optional)
        self.keep_alive_timer = QTimer()
        self.keep_alive_timer.timeout.connect(self.ping_broker)
        self.keep_alive_timer.start(30000)  # every 30 seconds

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # ----- Connection status -----
        self.status_label = QLabel("Connecting to broker...")
        main_layout.addWidget(self.status_label)

        # ----- Control groups -----
        control_layout = QHBoxLayout()

        # Group: Start / Stop
        start_stop_group = QGroupBox("Robot Control")
        ss_layout = QVBoxLayout()
        self.start_btn = QPushButton("START")
        self.start_btn.clicked.connect(self.send_start)
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.clicked.connect(self.send_stop)
        ss_layout.addWidget(self.start_btn)
        ss_layout.addWidget(self.stop_btn)
        start_stop_group.setLayout(ss_layout)
        control_layout.addWidget(start_stop_group)

        # Group: PID Tuning
        pid_group = QGroupBox("PID Parameters")
        pid_layout = QGridLayout()
        pid_layout.addWidget(QLabel("Kp:"), 0, 0)
        self.kp_spin = QDoubleSpinBox()
        self.kp_spin.setRange(0, 1)
        self.kp_spin.setSingleStep(0.01)
        self.kp_spin.setValue(0.04)
        self.kp_spin.valueChanged.connect(self.send_kp)
        pid_layout.addWidget(self.kp_spin, 0, 1)

        pid_layout.addWidget(QLabel("Ki:"), 1, 0)
        self.ki_spin = QDoubleSpinBox()
        self.ki_spin.setRange(0, 1)
        self.ki_spin.setSingleStep(0.01)
        self.ki_spin.setValue(0.0)
        self.ki_spin.valueChanged.connect(self.send_ki)
        pid_layout.addWidget(self.ki_spin, 1, 1)

        pid_layout.addWidget(QLabel("Kd:"), 2, 0)
        self.kd_spin = QDoubleSpinBox()
        self.kd_spin.setRange(0, 1)
        self.kd_spin.setSingleStep(0.01)
        self.kd_spin.setValue(0.1)
        self.kd_spin.valueChanged.connect(self.send_kd)
        pid_layout.addWidget(self.kd_spin, 2, 1)
        pid_group.setLayout(pid_layout)
        control_layout.addWidget(pid_group)

        # Group: Base speed
        speed_group = QGroupBox("Base Speed")
        speed_layout = QVBoxLayout()
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(12)  # 0.12 -> 12%
        self.speed_slider.valueChanged.connect(self.send_speed)
        self.speed_label = QLabel("0.12")
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_label)
        speed_group.setLayout(speed_layout)
        control_layout.addWidget(speed_group)

        # Group: Single commands
        cmd_group = QGroupBox("Single Commands")
        cmd_layout = QHBoxLayout()
        self.left_btn = QPushButton("LEFT")
        self.left_btn.clicked.connect(lambda: self.send_command("LEFT"))
        self.right_btn = QPushButton("RIGHT")
        self.right_btn.clicked.connect(lambda: self.send_command("RIGHT"))
        self.straight_btn = QPushButton("STRAIGHT")
        self.straight_btn.clicked.connect(lambda: self.send_command("STRAIGHT"))
        cmd_layout.addWidget(self.left_btn)
        cmd_layout.addWidget(self.right_btn)
        cmd_layout.addWidget(self.straight_btn)
        cmd_group.setLayout(cmd_layout)
        control_layout.addWidget(cmd_group)

        main_layout.addLayout(control_layout)

        # ----- Route planning -----
        route_group = QGroupBox("Plan Route")
        route_layout = QHBoxLayout()
        route_layout.addWidget(QLabel("From node:"))
        self.from_node = QSpinBox()
        self.from_node.setRange(0, 13)
        self.from_node.setValue(0)
        route_layout.addWidget(self.from_node)
        route_layout.addWidget(QLabel("To node:"))
        self.to_node = QSpinBox()
        self.to_node.setRange(0, 13)
        self.to_node.setValue(3)  # e.g. Reuzenrad
        route_layout.addWidget(self.to_node)
        self.send_route_btn = QPushButton("Send Route")
        self.send_route_btn.clicked.connect(self.send_route)
        route_layout.addWidget(self.send_route_btn)

        # Optional: raw action list input
        route_layout.addWidget(QLabel("OR raw actions:"))
        self.raw_actions = QLineEdit()
        self.raw_actions.setPlaceholderText("e.g. LEFT,RIGHT,STRAIGHT")
        route_layout.addWidget(self.raw_actions)
        self.send_raw_btn = QPushButton("Send Raw")
        self.send_raw_btn.clicked.connect(self.send_raw_actions)
        route_layout.addWidget(self.send_raw_btn)

        main_layout.addWidget(route_group)

        # ----- Status display (node info) -----
        info_group = QGroupBox("Robot State")
        info_layout = QVBoxLayout()
        self.node_label = QLabel("Current node: unknown")
        self.queue_label = QLabel("Queue length: 0")
        info_layout.addWidget(self.node_label)
        info_layout.addWidget(self.queue_label)
        info_group.setLayout(info_layout)
        main_layout.addWidget(info_group)

        # ----- Message log -----
        log_group = QGroupBox("Message Log")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

    # --------------------------------------------------------
    # MQTT callbacks
    # --------------------------------------------------------
    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            self.status_label.setText(f"Connected to {BROKER}")
            # Subscribe to incoming topics
            for topic in SUBSCRIBE_TOPICS:
                self.mqtt_client.subscribe(topic)
                self.log(f"Subscribed to {topic}")
        else:
            self.status_label.setText(f"Connection failed, code: {reason_code}")

    def on_disconnect(self, client, userdata, rc, properties=None):
        self.status_label.setText("Disconnected from broker")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()
        short_topic = topic.split("/")[-1]
        self.log(f"{short_topic}: {payload}")

        # Update node info if it's the node topic
        if topic == TOPIC_NODE:
            try:
                data = json.loads(payload)
                node = data.get("node", "?")
                queue = data.get("queue", 0)
                self.node_label.setText(f"Current node: {node}")
                self.queue_label.setText(f"Queue length: {queue}")
            except:
                pass

    # --------------------------------------------------------
    # Publishing methods
    # --------------------------------------------------------
    def send_start(self):
        self.mqtt_client.publish(TOPIC_CMD, "START")
        self.log("Sent: START")

    def send_stop(self):
        self.mqtt_client.publish(TOPIC_CMD, "STOP")
        self.log("Sent: STOP")

    def send_kp(self, value):
        self.mqtt_client.publish(TOPIC_PID_KP, str(value))
        self.log(f"Sent Kp = {value:.3f}")

    def send_ki(self, value):
        self.mqtt_client.publish(TOPIC_PID_KI, str(value))
        self.log(f"Sent Ki = {value:.3f}")

    def send_kd(self, value):
        self.mqtt_client.publish(TOPIC_PID_KD, str(value))
        self.log(f"Sent Kd = {value:.3f}")

    def send_speed(self, value):
        speed = value / 100.0
        self.speed_label.setText(f"{speed:.2f}")
        self.mqtt_client.publish(TOPIC_SPEED, str(speed))
        self.log(f"Sent base speed = {speed:.2f}")

    def send_command(self, direction):
        self.mqtt_client.publish(TOPIC_COMMAND, direction)
        self.log(f"Sent command: {direction}")

    def send_route(self):
        from_node = self.from_node.value()
        to_node = self.to_node.value()
        route_json = json.dumps({"from": from_node, "to": to_node})
        self.mqtt_client.publish(TOPIC_ROUTE, route_json)
        self.log(f"Sent route: {from_node} -> {to_node}")

    def send_raw_actions(self):
        raw = self.raw_actions.text().strip()
        if raw:
            # Expect comma separated list, e.g. "LEFT,RIGHT,STRAIGHT"
            actions = [a.strip().upper() for a in raw.split(",")]
            # Validate
            valid = all(a in ("LEFT", "RIGHT", "STRAIGHT") for a in actions)
            if valid:
                route_json = json.dumps(actions)
                self.mqtt_client.publish(TOPIC_ROUTE, route_json)
                self.log(f"Sent raw actions: {actions}")
            else:
                self.log("Invalid raw actions – use LEFT/RIGHT/STRAIGHT only")

    def ping_broker(self):
        # Optional: publish a small debug message to keep connection alive
        # Not strictly needed, but can be useful
        self.mqtt_client.publish(TOPIC_DEBUG, "dashboard ping")

    def log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")

    def closeEvent(self, event):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        event.accept()

# ------------------------------------------------------------
# Main
# ------------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    dashboard = RobotDashboard()
    dashboard.show()
    sys.exit(app.exec_())
