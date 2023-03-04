import time
from gpiozero import PhaseEnableMotor


left_motor_dir = 26
left_motor_pwm = 13

right_motor_dir = 6
right_motor_pwm = 5

med_bot_L = PhaseEnableMotor(left_motor_dir, left_motor_pwm)
med_bot_R = PhaseEnableMotor(right_motor_dir, right_motor_pwm)

med_bot_R.forward()
time.sleep(5)