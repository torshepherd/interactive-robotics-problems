import matplotlib.pyplot as plt, numpy as np

def generate_cubic_function(duration, start_point, end_point, start_dot=0, end_dot=0):
    # cubic(t)           => c1(t)**3 + c2(t)**2 + c3t + c4
    # start_point at t=0 => c4 = start_point
    # first derivative   => 3c1(t)**2 + 2c2t + c3
    # zero at t=0        => c3 = 0
    # zero at t=duration   => c2 = -1.5c1(duration)

    # cubic(t)              => c1(t)**3 - 1.5c1(duration)(t)**2 + start_point
    # end_point at t=duration => c1 = -2 (end_point - start_point) / (duration)**3

    c1 = -2 * (end_point - start_point) / ((duration)**3)
    c2 = -1.5 * c1 * (duration)
    c3 = 0
    c4 = start_point

    return lambda t: c1 * (t**3) + c2 * (t**2) + c3 * t + c4
