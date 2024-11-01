import RPi.GPIO as GPIO
import time

PWMA = 18
PWMB = 23
AIN1 = 22
AIN2 = 27
BIN1 = 25
BIN2 = 24

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)

L_Motor = GPIO.PWM(PWMA, 500)
R_Motor = GPIO.PWM(PWMB, 500)
L_Motor.start(0)
R_Motor.start(0)

try:
    while True:
        GPIO.output(AIN1, 1)
        GPIO.output(AIN2, 0)
        L_Motor.ChangeDutyCycle(100)
        
        GPIO.output(BIN1, 1)
        GPIO.output(BIN2, 0)
        R_Motor.ChangeDutyCycle(100)

        time.sleep(1.0)

        L_Motor.ChangeDutyCycle(0)
        R_Motor.ChangeDutyCycle(0)
        time.sleep(1.0)

except KeyboardInterrupt:
    pass

L_Motor.stop()
R_Motor.stop()
GPIO.cleanup()
