import RPi.GPIO as GPIO
import time

right_motor_dir = 13
right_motor_pwm = 19

left_motor_dir = 5
left_motor_pwm = 6

GPIO.setmode(GPIO.BCM)

GPIO.setup(right_motor_dir , GPIO.OUT)
GPIO.setup(right_motor_pwm , GPIO.OUT)

GPIO.setup(left_motor_dir , GPIO.OUT)
GPIO.setup(left_motor_pwm , GPIO.OUT)

pwm_r = GPIO.PWM(right_motor_pwm , 1000)
pwm_l = GPIO.PWM(left_motor_pwm , 1000)

pwm_r.start(100)
pwm_l.start(100)

def forward(second):
    print("forward")
    GPIO.output(right_motor_dir,GPIO.HIGH)
    GPIO.output(left_motor_dir,GPIO.HIGH)
    time.sleep(second)

def reverse(second):
    print("reverse")
    GPIO.output(right_motor_dir,GPIO.LOW)
    GPIO.output(left_motor_dir,GPIO.LOW)
    time.sleep(second)

def left(second):
    print("left")
    GPIO.output(right_motor_dir,GPIO.HIGH)
    GPIO.output(left_motor_dir,GPIO.LOW)
    time.sleep(second)

def right(second):
    print("right")
    GPIO.output(right_motor_dir,GPIO.LOW)
    GPIO.output(left_motor_dir,GPIO.HIGH)
    time.sleep(second)

def stop(second):
    print("stop")
    pwm_r.ChangeDutyCycle(0)
    pwm_l.ChangeDutyCycle(0)

def exit():
    GPIO.cleanup()

def main():
    forward(5)
    reverse(5)
    left(5)
    right(5)
    stop(5)
    exit()

if __name__ == '__main__':
    main()