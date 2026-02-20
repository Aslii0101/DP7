from machine import Pin, PWM, ADC
import time

# CONSTANTS
STOP = 0
SLOW = 16384
MEDIUM = 32768
FAST = 49152
MAX_SPEED = 65535

DREMPEL = 30000

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


def forward(speed=MEDIUM):
    dir_a.value(1)
    dir_b.value(0)
    pwm_a.duty_u16(speed)
    pwm_b.duty_u16(speed)


def backwards(speed=MEDIUM):
    dir_a.value(0)
    dir_b.value(1)
    pwm_a.duty_u16(speed)
    pwm_b.duty_u16(speed)


def sharp_left():
    dir_a.value(0)
    dir_b.value(0)
    pwm_a.duty_u16(SLOW)
    pwm_b.duty_u16(FAST)


def sharp_right():
    dir_a.value(1)
    dir_b.value(1)
    pwm_a.duty_u16(FAST)
    pwm_b.duty_u16(SLOW)


def turn_left():
    dir_a.value(1)
    dir_b.value(0)
    pwm_a.duty_u16(SLOW)
    pwm_b.duty_u16(MEDIUM)


def turn_right():
    dir_a.value(1)
    dir_b.value(0)
    pwm_a.duty_u16(MEDIUM)
    pwm_b.duty_u16(SLOW)


def search_pattern():
    print("Search")
    sharp_left()
    time.sleep(0.3)
    stop_motors()


def test_movement():
    print("Running movement tests")
    try:
        print("Forward")
        forward()
        time.sleep(0.5)
        stop_motors()
        time.sleep(0.2)

        print("Backwards")
        backwards()
        time.sleep(0.5)
        stop_motors()
        time.sleep(0.2)

        print("Left")
        turn_left()
        time.sleep(0.5)
        stop_motors()
        time.sleep(0.2)

        print("Right")
        turn_right()
        time.sleep(0.5)
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
    # test_sensors()
    test_movement()
