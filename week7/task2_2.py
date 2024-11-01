import RPi.GPIO as GPIO
import time

SWITCH_list = [5, 6, 13, 19]
BUZZER = 12

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

for pin in SWITCH_list:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(BUZZER, GPIO.OUT)

p = GPIO.PWM(BUZZER, 261.63)
p.start(0)

try:
    while True:
        if GPIO.input(SWITCH_list[0]) == 1:
            p.ChangeDutyCycle(50)
            p.ChangeFrequency(261.63)
            time.sleep(0.1)
            p.ChangeDutyCycle(0)
            print("C")
        elif GPIO.input(SWITCH_list[1]) == 1:
            p.ChangeDutyCycle(50)
            p.ChangeFrequency(293.66)
            time.sleep(0.1)
            p.ChangeDutyCycle(0)
            print("D")
        elif GPIO.input(SWITCH_list[2]) == 1:
            p.ChangeDutyCycle(50)
            p.ChangeFrequency(329.63)
            time.sleep(0.1)
            p.ChangeDutyCycle(0)
            print("E")
        elif GPIO.input(SWITCH_list[3]) == 1:
            p.ChangeDutyCycle(50)
            p.ChangeFrequency(392)
            time.sleep(0.1)
            p.ChangeDutyCycle(0)
            print("G")
            
        time.sleep(0.1)

except KeyboardInterrupt:
    pass

p.stop()
GPIO.cleanup()