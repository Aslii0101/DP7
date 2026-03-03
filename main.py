from machine import Pin, PWM, ADC
import time

# ================= MOTORS =================
motor1_pwm = PWM(Pin(17), freq=1000)   # fysiek gespiegeld
motor1_in1 = Pin(13, Pin.OUT)

motor2_pwm = PWM(Pin(11), freq=1000)
motor2_in1 = Pin(12, Pin.OUT)

# ================= SENSORS =================
pins = [1,7,6,5,4]
sensors = [ADC(Pin(p), atten=ADC.ATTN_11DB) for p in pins]

THRESHOLD = 30000

BASE_SPEED = 0.40
TURN_SPEED = 0.30
SHARP_SPEED = 0.24
SEARCH_SPEED = 0.28

# ================= MOTOR CONTROL =================
def set_speed(left, right):
    # ⭐ MOTORS WAREN OMGEKEERD
    motor1_pwm.duty_u16(int(65535 * right))
    motor2_pwm.duty_u16(int(65535 * left))
    motor1_in1.value(1)
    motor2_in1.value(0)

def stop():
    set_speed(0,0)

def forward():
    set_speed(BASE_SPEED, BASE_SPEED)

def soft_left():
    set_speed(TURN_SPEED*0.6, TURN_SPEED)

def soft_right():
    set_speed(TURN_SPEED, TURN_SPEED*0.6)

def sharp_left():
    set_speed(0.05, SHARP_SPEED)

def sharp_right():
    set_speed(SHARP_SPEED, 0.05)

def spin_left():
    set_speed(0, SEARCH_SPEED)

def spin_right():
    set_speed(SEARCH_SPEED, 0)

# ================= STATUS =================
last_direction = 1
lost_counter = 0

print("=== PERFECTE DP7 LINE FOLLOWER ===")

# ================= LOOP =================
while True:

    vals = [s.read_u16() for s in sensors]

    L1 = vals[0] < THRESHOLD
    L2 = vals[1] < THRESHOLD
    M  = vals[2] < THRESHOLD
    R2 = vals[3] < THRESHOLD
    R1 = vals[4] < THRESHOLD

    line_seen = L1 or L2 or M or R2 or R1

    print("Sensors:",L1,L2,M,R2,R1)

    # ================= LINE FOLLOW =================
    if line_seen:
        lost_counter = 0

        # scherpe bochten eerst
        if L1:
            print("SCHERP LINKS")
            sharp_left()
            last_direction = -1

        elif R1:
            print("SCHERP RECHTS")
            sharp_right()
            last_direction = 1

        # lichte bochten
        elif L2:
            print("LICHT LINKS")
            soft_left()
            last_direction = -1

        elif R2:
            print("LICHT RECHTS")
            soft_right()
            last_direction = 1

        # recht maar corrigeren
        elif M:
            print("RECHT + CORRECTIE")
            forward()

    # ================= SEARCH MODE =================
    else:
        lost_counter += 1
        print("LIJN KWIJT:", lost_counter)

        # ⭐ eerst stoppen
        if lost_counter == 1:
            stop()
            time.sleep(0.15)

        # scan laatste richting
        elif lost_counter < 40:
            if last_direction == 1:
                spin_right()
            else:
                spin_left()

        # volledige draai
        elif lost_counter < 100:
            print("180° ZOEK")
            spin_right()

        # reset poging
        else:
            stop()
            time.sleep(0.2)
            lost_counter = 0

    time.sleep(0.02)