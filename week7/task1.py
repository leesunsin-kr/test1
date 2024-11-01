import RPi.GPIO as GPIO
import time

SWITCH_PINS = [5, 6, 13, 19] 
click_count = [0, 0, 0, 0]    
prev_state = [0, 0, 0, 0]    

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

for pin in SWITCH_PINS:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    while True:
        for i in range(4):  
            current_state = GPIO.input(SWITCH_PINS[i])  
            
            if current_state == 1 and prev_state[i] == 0:  
                click_count[i] += 1
                if  SWITCH_PINS[i] == 5: 
                    print("SW1 click", click_count[i])
                elif  SWITCH_PINS[i] == 6: 
                    print("SW2 click", click_count[i])
                elif  SWITCH_PINS[i] == 13: 
                    print("SW3 click", click_count[i])
                if  SWITCH_PINS[i] == 19: 
                    print("SW4 click", click_count[i])
            prev_state[i] = current_state 

        time.sleep(0.1) 

except KeyboardInterrupt:
    pass

GPIO.cleanup()
