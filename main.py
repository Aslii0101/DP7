from machine import Pin, PWM, ADC
import time

sensorL1 = ADC(Pin(1), atten=ADC.ATTN_11DB)
sensorL2 = ADC(Pin(7), atten=ADC.ATTN_11DB)
sensorM = ADC(Pin(6), atten=ADC.ATTN_11DB)
sensorR2 = ADC(Pin(5), atten=ADC.ATTN_11DB)
sensorR1 = ADC(Pin(4), atten=ADC.ATTN_11DB)

pwm_a = PWM(Pin(17), freq=1000)
dir_a = Pin(13, Pin.OUT)


while True:
    time.sleep(1)
    line_values = [
        sensorL1.read_u16(),
        sensorL2.read_u16(),
        sensorM.read_u16(),
        sensorR2.read_u16(),
        sensorR1.read_u16(),
    ]

    print(line_values)

    dir_a.value(1)
    pwm_a.duty_u16(32768)

    time.sleep(2)

    pwm_a.duty_u16(0)
