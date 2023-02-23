import time
from gpiozero import PhaseEnableMotor


left_motor_dir = 17
left_motor_pwm = 27

right_motor_dir = 16
right_motor_pwm = 26

med_bot_L = PhaseEnableMotor(left_motor_dir, left_motor_pwm)
med_bot_R = PhaseEnableMotor(right_motor_dir, right_motor_pwm)

for i in range(4):
    med_bot_L.forward()
    time.sleep(5)
    med_bot_L.backward()
    time.sleep(5)