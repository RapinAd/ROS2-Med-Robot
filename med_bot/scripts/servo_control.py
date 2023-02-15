import RPi.GPIO as GPIO
import time

servo = 12

GPIO.setmode(GPIO.BCM)

GPIO.setup(servo, GPIO.OUT)

servo1 = GPIO.PWM(servo , 50)

servo1.start(0)

duty = 12

while duty >= 8.65:
    servo1.ChangeDutyCycle(duty)
    duty = duty - 0.1
    time.sleep(0.1)

time.sleep(2)

while duty <= 12:
    servo1.ChangeDutyCycle(duty)
    duty = duty + 0.1
    time.sleep(0.1)


servo1.stop()
GPIO.cleanup()