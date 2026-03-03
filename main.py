from machine import Pin, PWM, ADC
import time
import urandom

# motors
motor1_pwm = PWM(Pin(17), freq=1000)
motor1_in1 = Pin(13, Pin.OUT)

motor2_pwm = PWM(Pin(11), freq=1000)
motor2_in1 = Pin(12, Pin.OUT)

# sensors
pins = [1,7,6,5,4]
sensors = [ADC(Pin(p), atten=ADC.ATTN_11DB) for p in pins]

THRESHOLD = 30000

# speeds
BASE_SPEED = 0.42
TURN_SPEED = 0.32
SHARP_SPEED = 0.26
SEARCH_SPEED = 0.30

# motor control
def set_speed(left, right):
    motor1_pwm.duty_u16(int(65535 * right))
    motor2_pwm.duty_u16(int(65535 * left))
    motor1_in1.value(1)
    motor2_in1.value(0)

def stop():
    set_speed(0, 0)

def forward():
    set_speed(BASE_SPEED, BASE_SPEED)

def soft_left():
    set_speed(TURN_SPEED * 0.6, TURN_SPEED)

def soft_right():
    set_speed(TURN_SPEED, TURN_SPEED * 0.6)

def sharp_left():
    set_speed(0.05, SHARP_SPEED)

def sharp_right():
    set_speed(SHARP_SPEED, 0.05)

def spin_left():
    set_speed(0, SEARCH_SPEED)

def spin_right():
    set_speed(SEARCH_SPEED, 0)

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
    M  = vals[2] < THRESHOLD
    R2 = vals[3] < THRESHOLD
    R1 = vals[4] < THRESHOLD

    line_seen = L1 or L2 or M or R2 or R1

    # T-kruispunt (alles zwart)
    if L1 and L2 and M and R2 and R1:
        robot_state = "T-KRUISPUNT"
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

    # lijn volgen
    elif line_seen:
        lost_counter = 0

        if L1:
            sharp_left()
            last_direction = -1
            robot_state = "SCHERP LINKS"

        elif R1:
            sharp_right()
            last_direction = 1
            robot_state = "SCHERP RECHTS"

        elif L2:
            soft_left()
            last_direction = -1
            robot_state = "LICHT LINKS"

        elif R2:
            soft_right()
            last_direction = 1
            robot_state = "LICHT RECHTS"

        elif M:
            forward()
            robot_state = "RECHT"

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

    # serial output
    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) > 1000:
        print(robot_state)
        last_print = now

    time.sleep(0.02)