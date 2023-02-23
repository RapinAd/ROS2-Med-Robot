import time
import numpy as np
from gpiozero import RotaryEncoder
import RPi.GPIO as GPIO
 
# Assigning parameter values
ppr = 400  # Pulses Per Revolution of the encoder
tstop = 20  # Loop execution duration (s)
tsample = 0.02  # Sampling period for code execution (s)
tdisp = 0.5  # Sampling period for values display (s)




encoder = RotaryEncoder(17, 27, max_steps=0)

# Initializing previous values and starting main clock
anglecurr = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()

# Execution loop that displays the current
# angular position of the encoder shaft
print('Running code for', tstop, 'seconds ...')
print('(Turn the encoder.)')
while True:
    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    # Getting angular position of the encoder
    # roughly every `tsample` seconds (deg.)
    anglecurr = 360 / ppr * encoder.steps
    # Printing angular position every `tdisp` seconds
    if (np.floor(tcurr/tdisp) - np.floor(tprev/tdisp)) == 1:
        print("Angle = {:0.0f} deg".format(anglecurr))
    # Updating previous values
    tprev = tcurr

print('Done.')
# Releasing GPIO pins
encoder.close()