# Importing modules and classes
import time
import numpy as np
from gpiozero_extended import Motor, PID

## Front Right Motor


# Setting general parameters
tstop = 1000  # Execution duration (s)
tsample = 0.01  # Sampling period (s)
thetamax = 180  # Motor position amplitude (deg)


mymotor = Motor(
    enable1=26, pwm1=7, pwm2=8,
    encoder1=19, encoder2=13, encoderppr=300.8)
mymotor.reset_angle()

# Initializing variables and starting clock
thetaprev = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()

# Running execution loop
print('Running code for', tstop, 'seconds ...')
while tcurr <= tstop:
    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    # Getting motor shaft angular position
    thetacurr = mymotor.get_angle()
    print(thetacurr)


print('Done.')
# Stopping motor and releasing GPIO pins
mymotor.set_output(0, brake=True)
del mymotor
