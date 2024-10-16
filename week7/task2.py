import RPi.GPIO as GPIO
import time

BUZZER = 12

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER, GPIO.OUT)

p = GPIO.PWM(BUZZER, 261)
p.start(50)  

try:
    while True:
        p.start(50)
        p.ChangeFrequency(261.63) 
        time.sleep(1.0)
        p.ChangeFrequency(293.66)  
        time.sleep(1.0)
        p.ChangeFrequency(329.63)  
        time.sleep(1.0)
        p.ChangeFrequency(349.23)  
        time.sleep(1.0)
        p.ChangeFrequency(392.00)  
        time.sleep(1.0)
        p.ChangeFrequency(440.00) 
        time.sleep(1.0)
        p.ChangeFrequency(493.88) 
        time.sleep(1.0)
        p.ChangeFrequency(523.25) 
        time.sleep(1.0)

except KeyboardInterrupt:
    pass

p.stop()
GPIO.cleanup()