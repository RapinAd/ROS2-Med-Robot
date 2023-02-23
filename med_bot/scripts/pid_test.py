import numpy as np
import time

from gpiozero import PhaseEnableMotor, RotaryEncoder

# Define Pin
left_motor_dir = 5
left_motor_pwm = 6

right_motor_dir = 16
right_motor_pwm = 26

left_encoder_a = 17
left_encoder_b = 27

right_encoder_a = 23
right_encoder_b = 24

# Pin Enable
med_bot_L = PhaseEnableMotor(left_motor_dir, left_motor_pwm)
med_bot_R = PhaseEnableMotor(right_motor_dir, right_motor_pwm)
encoderL = RotaryEncoder(left_encoder_a, left_encoder_b, max_steps=0)
encoderR = RotaryEncoder(right_encoder_a, right_encoder_b, max_steps=0)

# Assigning parameter values
ppr = 400  # Pulses Per Revolution of the encoder
tstop = 20  # Loop execution duration (s)
tsample = 0.02  # Sampling period for code execution (s)
target_L = -83
target_R = 83
kp = 0.1
ki = 15


# Global Variable
tprev_L = 0
tcurr_L = 0
tdiff_L = 0
step_curr_L = 0
step_prev_L = 0
step_diff_L = 0
rpm_L = 0.0
rpm_L_fil = 0.0
rpm_L_prev = 0.0
eintegral_L = 0.0

tprev_R = 0
tcurr_R = 0
tdiff_R = 0
step_curr_R = 0
step_prev_R = 0
step_diff_R = 0
rpm_R = 0.0
rpm_R_fil = 0.0
rpm_R_prev = 0.0
eintegral_R = 0.0

tstart = time.perf_counter()

# loop
while tcurr_L <= tstop and tcurr_R <= tstop:
    time.sleep(tsample)
    if tcurr_L <= tstop/3:
        target_L = -83
        target_R = 83
    elif tcurr_L <= 2*tstop/3:
        target_L = 83
        target_R = 83
    else:
        target_L = 83
        target_R = -83
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

    # low-pass filter 25 Hz
    rpm_L_fil = 0.854*rpm_L_fil + 0.0728*rpm_L + 0.0728*rpm_L_prev
    rpm_L_prev = rpm_L

    rpm_R_fil = 0.854*rpm_R_fil + 0.0728*rpm_R + 0.0728*rpm_R_prev
    rpm_R_prev = rpm_R

    # signal u
    eL = target_L - rpm_L_fil
    eR = target_R - rpm_R_fil

    eintegral_L = eintegral_L + eL*tdiff_L
    eintegral_R = eintegral_R + eR*tdiff_R

    uL = (kp*eL + ki*eintegral_L)/255
    uR = (kp*eR + ki*eintegral_R)/255

    print(target_L, "       |       ", target_R)
    print(rpm_L_fil, "       |       ", rpm_R_fil)



    if uL > 0:
        if uL > 1:
            uL = 1
        med_bot_L.forward(uL)
    else:
        uL = -uL
        if uL > 1:
            uL = 1
        med_bot_L.backward(uL)

    if uR > 0:
        if uR > 1:
            uR = 1
        med_bot_R.forward(uR)
    else:
        uR = -uR
        if uR > 1:
            uR = 1
        med_bot_R.backward(uR)
