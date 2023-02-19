import time
from gpiozero import PhaseEnableRobot


left_motor_dir = 13
left_motor_pwm = 19

right_motor_dir = 5
right_motor_pwm = 6

med_bot = PhaseEnableRobot(left=(left_motor_dir, left_motor_pwm), right=(
    right_motor_dir, right_motor_pwm))

for i in range(4):
    med_bot.forward()
    time.sleep(5)
    med_bot.right()
    time.sleep(5)