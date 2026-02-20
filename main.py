from machine import Pin, PWM, ADC
import time

sensorL1 = ADC(Pin(1), atten=ADC.ATTN_11DB)
sensorL2 = ADC(Pin(7), atten=ADC.ATTN_11DB)
sensorM = ADC(Pin(6), atten=ADC.ATTN_11DB)
sensorR2 = ADC(Pin(5), atten=ADC.ATTN_11DB)
sensorR1 = ADC(Pin(4), atten=ADC.ATTN_11DB)

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
