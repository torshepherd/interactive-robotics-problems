import robottools as rt
import numpy as np
import pygame
from time import time
from robottools import PID_control, integrate_dynamics
from numpy import pi
import mmap
import struct

FALL_LIMIT = (2 * pi / 3)
REFERENCE_LIMIT = 1 * pi

# Instances
# mass_beam, mass_wheel, inertia_beam, inertia_wheel, length_center_of_mass, radius_wheel
B = rt.Balancebot(1, 0.06, 0.006, 0.000048, .1, 0.04)

# Kp, Kd, Ki
D1 = rt.PID(-2.5, -.2, -.1)
D2 = rt.PID(.02, 0.000, 0.003)

done = False

prev_time = time()
states = np.array([[0.02],
                   [0],
                   [0],
                   [0]])
inputs = 0
integrated_e_1 = 0
previous_e_1 = 0
integrated_e_2 = 0
previous_e_2 = 0
x3_ref = 0

with open("state.tmp", "r+b") as f_s, open("ref.tmp", "r+b") as f_r, open("input.tmp", "r+b") as f_i:
    state_mm = mmap.mmap(f_s.fileno(), 19)
    reference_mm = mmap.mmap(f_r.fileno(), 4)
    input_mm = mmap.mmap(f_i.fileno(), 4)

    while not done:
        # Compute time difference
        time_diff = time() - prev_time
        prev_time = time()
        if time_diff == 0:
            time_diff = 0.001

        for i in range(4):
            data = state_mm.readline().rstrip()
            try:
                states[i, 0] = struct.unpack('f', data)[0]
            except:
                print('Input not read properly.')
        state_mm.seek(0)

        # Limit wheel angle reference to avoid instability
        if x3_ref - states[2, 0] > REFERENCE_LIMIT:
            x3_ref_sat = states[2, 0] + REFERENCE_LIMIT
        elif x3_ref - states[2, 0] < -REFERENCE_LIMIT:
            x3_ref_sat = states[2, 0] - REFERENCE_LIMIT
        else:
            x3_ref_sat = x3_ref

        # Run PID control on wheel angle -> body angle
        x1_ref, integrated_e_2, previous_e_2 = PID_control(x3_ref_sat,
                                                           states[2, 0],
                                                           integrated_e_2,
                                                           previous_e_2,
                                                           time_diff,
                                                           D2)

        # Run PID control on body angle -> motor torque
        inputs, integrated_e_1, previous_e_1 = PID_control(x1_ref,
                                                           states[0, 0],
                                                           integrated_e_1,
                                                           previous_e_1,
                                                           time_diff,
                                                           D1)

        input_mm[:] = struct.pack('f', inputs)

    state_mm.close()
    reference_mm.close()
    input_mm.close()

quit()
