from machine import Pin, PWM, ADC
import time

# CONSTANTS
STOP = 0
SLOW = 16384
MEDIUM = 32768
FAST = 49152
MAX_SPEED = 65535


pwm_a = PWM(Pin(17), freq=1000)
dir_a = Pin(13, Pin.OUT)
pwm_b = PWM(Pin(11), freq=1000)
dir_b = Pin(12, Pin.OUT)

pwm_a.duty_u16(0)
pwm_b.duty_u16(0)

sensorL1 = ADC(Pin(1), atten=ADC.ATTN_11DB)
sensorL2 = ADC(Pin(7), atten=ADC.ATTN_11DB)
sensorM = ADC(Pin(6), atten=ADC.ATTN_11DB)
sensorR2 = ADC(Pin(5), atten=ADC.ATTN_11DB)
sensorR1 = ADC(Pin(4), atten=ADC.ATTN_11DB)


def stop_motors():
    pwm_a.duty_u16(0)
    pwm_b.duty_u16(0)


def set_motor_a(speed, direction):
    dir_a.value(direction)
    pwm_a.duty_u16(speed)


def set_motor_b(speed, direction):
    dir_b.value(direction)
    pwm_b.duty_u16(speed)


def move_forward(speed):
    set_motor_a(speed, 1)
    set_motor_b(speed, 0)


def move_backwards(speed):
    set_motor_a(speed, 0)
    set_motor_b(speed, 1)


def turn_left(speed):
    set_motor_a(speed, 1)
    set_motor_b(0, 0)


def turn_right(speed):
    set_motor_a(0, 1)
    set_motor_b(speed, 0)


def test_movement():
    print("Running movement tests")
    try:
        print("Forward")
        move_forward(SLOW)
        time.sleep(0.5)
        stop_motors()
        time.sleep(0.2)

        print("Backwards")
        move_backwards(SLOW)
        time.sleep(0.5)
        stop_motors()
        time.sleep(0.2)

        print("Left")
        turn_left(SLOW)
        time.sleep(0.4)
        stop_motors()
        time.sleep(0.2)

        print("Right")
        turn_right(SLOW)
        time.sleep(0.4)
        stop_motors()
        time.sleep(0.2)

        print("Movement tests complete")
    except Exception as e:
        print("Movement test error:", e)


def test_sensors():
    line_values = [
        sensorL1.read_u16(),
        sensorL2.read_u16(),
        sensorM.read_u16(),
        sensorR2.read_u16(),
        sensorR1.read_u16(),
    ]

    print(line_values)


while True:
    test_sensors()
    test_movement()
