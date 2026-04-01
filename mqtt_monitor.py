import paho.mqtt.client as mqtt
import time
import random
import string
from datetime import datetime

BROKER = "mqtt.hbo-ict.net"
PORT = 1883
TOPIC = "student/AD25VBa4/#"
CLIENT_ID = "monitor_" + "".join(random.sample(string.ascii_letters, 8))

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        client.subscribe(TOPIC)
        print(f"[{timestamp()}] Connected to {BROKER}")
        print(f"[{timestamp()}] Subscribed to {TOPIC}")
        print("-" * 50)
    else:
        print(f"Connection failed, code: {reason_code}")

def on_message(client, userdata, msg):
    topic = msg.topic.split("/")[-1]  # last part of topic, e.g. "robot_status"
    payload = msg.payload.decode()
    print(f"[{timestamp()}] {topic}: {payload}")

def timestamp():
    return datetime.now().strftime("%H:%M:%S")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, CLIENT_ID)
client.on_connect = on_connect
client.on_message = on_message

print("=== DP7 MQTT Monitor ===")
print("Connecting to broker...")
client.connect(BROKER, PORT)
client.loop_start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopping...")
    client.loop_stop()
    client.disconnect()
