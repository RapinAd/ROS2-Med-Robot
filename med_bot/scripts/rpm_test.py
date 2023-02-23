import numpy as np
import time
import plotext as plt

from gpiozero import PhaseEnableMotor , RotaryEncoder

#Define Pin
left_motor_dir = 16
left_motor_pwm = 26

right_motor_dir = 5
right_motor_pwm = 6

left_encoder_a = 17
left_encoder_b = 27

right_encoder_a = 23
right_encoder_b = 24

#Pin Enable

encoderL = RotaryEncoder(left_encoder_a, left_encoder_b, max_steps=0)
encoderR = RotaryEncoder(right_encoder_a, right_encoder_b, max_steps=0)
med_bot_R = PhaseEnableMotor(right_motor_dir, right_motor_pwm)
med_bot_L = PhaseEnableMotor(left_motor_dir, left_motor_pwm)

# Assigning parameter values
ppr = 400  # Pulses Per Revolution of the encoder
tstop = 20  # Loop execution duration (s)
tsample = 0.05  # Sampling period for code execution (s)


#Global Variable
tprev_L = 0
tcurr_L = 0
tdiff_L = 0
step_curr_L = 0
step_prev_L = 0
step_diff_L = 0
rpm_L = 0.0

tprev_R = 0
tcurr_R = 0
tdiff_R = 0
step_curr_R = 0
step_prev_R = 0
step_diff_R = 0
rpm_R = 0.0

tstart = time.perf_counter()

#loop
while True:
    time.sleep(tsample)

    step_curr_L = encoderL.steps
    tcurr_L = time.perf_counter() - tstart
    tdiff_L = tcurr_L - tprev_L
    step_diff_L = step_curr_L-step_prev_L
    rpm_L = (-step_diff_L / tdiff_L) * 60 / ppr
    step_prev_L = step_curr_L
    tprev_L = tcurr_L

    step_curr_R = encoderR.steps
    tcurr_R = time.perf_counter() - tstart
    tdiff_R = tcurr_R - tprev_R
    step_diff_R = step_curr_R-step_prev_R
    rpm_R = (step_diff_R / tdiff_R) * 60 / ppr
    step_prev_R = step_curr_R
    tprev_R = tcurr_R


    # step_curr_L = encoderL.steps

    # if step_curr_L != step_prev_L:
    #     step_diff_L = step_curr_L-step_prev_L
    #     tcurr_L = time.perf_counter() - tstart
    #     tdiff_L = tcurr_L - tprev_L
    #     rpm_L = (-step_diff_L / tdiff_L) * 60 / ppr
    #     tprev_L = tcurr_L
    #     step_prev_L = step_curr_L
    # else:
    #     rpm_L = 0.0
    
    # step_curr_R = encoderR.steps

    # if step_curr_R != step_prev_R:
    #     step_diff_R = step_curr_R - step_prev_R
    #     tcurr_R = time.perf_counter() - tstart
    #     tdiff_R = tcurr_R - tprev_R
    #     rpm_R = (step_diff_R / tdiff_R) * 60 / ppr
    #     tprev_R = tcurr_R
    #     step_prev_R = step_curr_R
    # else:
    #     rpm_R = 0.0
    med_bot_L.forward()
    med_bot_R.forward()
    print(rpm_L, "       |       ",rpm_R)





