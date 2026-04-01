from machine import Pin, PWM, ADC
import time
import urandom
import network
from umqtt.simple import MQTTClient

# --- WiFi + MQTT config ---
WIFI_SSID = "hbo-ict-iot"
WIFI_PASS = "wateenleukestudie!"
BROKER = "mqtt.hbo-ict.net"
PORT = 1883
TOPIC = b"student/AD25VBa4/robot_status"

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASS)
    print("Connecting to WiFi...")
    for _ in range(20):
        if wlan.isconnected():
            print("WiFi connected:", wlan.ifconfig()[0])
            return True
        time.sleep(0.5)
    print("WiFi failed, running without MQTT")
    return False

mqtt_client = None
if connect_wifi():
    try:
        mqtt_client = MQTTClient("dp7_robot_AD25VBa4", BROKER, PORT)
        mqtt_client.connect()
        print("MQTT connected")
    except Exception as e:
        print("MQTT connect failed:", e)
        mqtt_client = None

# motors
motor1_pwm = PWM(Pin(17), freq=60)
motor1_in1 = Pin(13, Pin.OUT)

motor2_pwm = PWM(Pin(11), freq=60)
motor2_in1 = Pin(12, Pin.OUT)

# sensors
pins = [1, 7, 6, 5, 4]
sensors = [ADC(Pin(p), atten=ADC.ATTN_11DB) for p in pins]

THRESHOLD = 30000

# speeds
BASE_SPEED = 0.30
SHARP_SPEED = 0.16
SEARCH_SPEED = 0.25

# PID
Kp, Ki, Kd = 0.6, 0.2, 0.1
previousError = 0
P, I, D = 0.0, 0.0, 0.0

def pid(error):
    global previousError, P, I, D
    P = error
    I = max(-10, min(10, I + error))
    D = error - previousError
    previousError = error
    output = Kp * P + Ki * I + Kd * D
    return output


# motor control
def set_speed(left, right):
    motor1_in1.value(1 if right >= 0 else 0)
    motor1_pwm.duty_u16(int(65535 * abs(right)))
    motor2_in1.value(0 if left >= 0 else 1)
    motor2_pwm.duty_u16(int(65535 * abs(left)))


def stop():
    set_speed(0, 0)


def forward():
    set_speed(BASE_SPEED, BASE_SPEED)



def sharp_left():
    set_speed(0.05, SHARP_SPEED)


def sharp_right():
    set_speed(SHARP_SPEED, 0.05)


def spin_left():
    set_speed(-SEARCH_SPEED, SEARCH_SPEED)


def spin_right():
    set_speed(SEARCH_SPEED, -SEARCH_SPEED)


# status
last_direction = 1
lost_counter = 0
robot_state = "START"

# print timer (1x per seconde)
last_print = time.ticks_ms()

print("=== DP7 LINE FOLLOWER START ===")

while True:

    vals = [s.read_u16() for s in sensors]

    L1 = vals[0] < THRESHOLD
    L2 = vals[1] < THRESHOLD
    M = vals[2] < THRESHOLD
    R2 = vals[3] < THRESHOLD
    R1 = vals[4] < THRESHOLD

    line_seen = L1 or L2 or M or R2 or R1

    # T-kruispunt (alles zwart)
    if L1 and L2 and M and R2 and R1:
        robot_state = "T-KRUISPUNT"
        forward()
        time.sleep(0.1)
        if urandom.getrandbits(1):
            sharp_left()
            last_direction = -1
            robot_state += " -> LINKS"
        else:
            sharp_right()
            last_direction = 1
            robot_state += " -> RECHTS"
        time.sleep(0.25)

    # normaal kruispunt
    elif L2 and M and R2:
        forward()
        robot_state = "KRUISPUNT RECHTDOOR"

    # lijn volgen met PID
    elif line_seen:
        lost_counter = 0

        weights = [-2, -1, 0, 1, 2]
        active = [L1, L2, M, R2, R1]
        active_count = sum(active)
        error = sum(w for w, a in zip(weights, active) if a) / active_count

        correction = pid(error)
        left_speed = max(0.0, min(1.0, BASE_SPEED + correction))
        right_speed = max(0.0, min(1.0, BASE_SPEED - correction))
        set_speed(left_speed, right_speed)

        if error < -0.3:
            last_direction = -1
            robot_state = "LINKS ({:.2f})".format(error)
        elif error > 0.3:
            last_direction = 1
            robot_state = "RECHTS ({:.2f})".format(error)
        else:
            robot_state = "RECHT ({:.2f})".format(error)

    # zoeken
    else:
        lost_counter += 1

        if lost_counter < 60:
            if last_direction == 1:
                spin_right()
                robot_state = "ZOEKEN RECHTS"
            else:
                spin_left()
                robot_state = "ZOEKEN LINKS"
        else:
            spin_right()
            robot_state = "180° ZOEKEN"

    # serial + MQTT output
    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) > 1000:
        print(robot_state)
        if mqtt_client:
            try:
                mqtt_client.publish(TOPIC, robot_state.encode())
            except Exception as e:
                print("MQTT publish error:", e)
        last_print = now

    time.sleep(0.02)
