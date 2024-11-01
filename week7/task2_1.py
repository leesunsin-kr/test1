import RPi.GPIO as GPIO
import time

BUZZER = 12

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER,GPIO.OUT)

p = GPIO.PWM(BUZZER, 261)
p.start(50)

try:
    while True:
        p.start(50)
        p.ChangeFrequency(659)
        time.sleep(0.2)
        p.ChangeFrequency(698)
        time.sleep(0.2)
        p.ChangeFrequency(784)
        time.sleep(0.2)
        p.ChangeFrequency(659)
        time.sleep(0.2)
        p.ChangeFrequency(698)
        time.sleep(0.2)
        p.ChangeFrequency(784)
        time.sleep(0.2)
        p.ChangeFrequency(659)
        time.sleep(0.2)
        p.ChangeFrequency(698)
        time.sleep(0.2)
        p.ChangeFrequency(784)
        time.sleep(0.2)
        p.ChangeFrequency(988)
        time.sleep(0.2)
        p.stop()
        time.sleep(1.0)

except KeyboardInterrupt:
    pass

p.stop()
GPIO.cleanup()