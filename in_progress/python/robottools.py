import numpy as np
from queue import Queue
from threading import Thread
from numpy import sin, cos, pi

g = 9.81


class DoublePendulum:
    def __init__(self, mass_link_1, mass_link_2, inertia_1, inertia_2, length_1, length_2, f_static_1, f_static_2, f_viscous_1, f_viscous_2):
        self.m_1 = mass_link_1
        self.m_2 = mass_link_2
        self.I_1 = inertia_1
        self.I_2 = inertia_2
        self.l_1 = length_1
        self.l_2 = length_2
        self.l_c1 = self.l_1 / 2
        self.l_c2 = self.l_2 / 2
        self.fs_1 = f_static_1
        self.fs_2 = f_static_2
        self.fv_1 = f_viscous_1
        self.fv_2 = f_viscous_2
        
        self.d_22 = (self.m_2 * self.l_c2 ** 2) + self.I_2


class Balancebot:
    def __init__(self, mass_beam, mass_wheel, inertia_beam, inertia_wheel, length_center_of_mass, radius_wheel):
        self.m_b = mass_beam
        self.m_w = mass_wheel
        self.I_b = inertia_beam
        self.I_w = inertia_wheel
        self.L = length_center_of_mass
        self.R_w = radius_wheel

        self.a_1 = self.I_w + (self.m_b + self.m_w) * (self.R_w ** 2)
        self.a_2 = self.m_b * self.R_w * self.L
        self.a_3 = self.I_b + self.m_b * (self.L ** 2)
        self.a_4 = self.m_b * g * self.L


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

def wrap2pi(theta):
    return theta % (2 * pi)


def project_screen_2d(world_points, R, p, scale):
    return np.matmul(R * scale, (world_points + p / scale)).astype(int)


def project_world_2d(screen_points, R, p, scale):
    return np.matmul(np.linalg.inv(R * scale), (screen_points)) - p / scale


def integrate_dynamics(dynamics, states, inputs, dt, arg):
    return states + dt * dynamics(states, inputs, arg)


def polar2cart(r, theta):
    return r * np.array([[cos(theta)], [sin(theta)]])


def balancebot_dynamics(states, inputs, B):
    '''
    Calculate the time derivatives of [states] subject to [inputs] for the balancebot model.

    Arguments:
        states = numpy.array([[theta_1, theta_2, ..], 
                              [x_1, x_2, ..], 
                              [alpha_1, alpha_2, ..]])

            Must be numpy array of shape (1-3, 1-)

        inputs = numpy.array([[motor_1_1, motor_1_2, ..], 
                              [motor_2_1, motor_2_2, ..]])

            Must be numpy array of shape (2, 1-)

    Returns:
        dynamics = numpy.array([[theta_1, theta_2, ..], 
                                [x_1, x_2, ..], 
                                [alpha_1, alpha_2, ..]])

            numpy array of same shape as [states]

    The number of rows in [states] determines the model used. With one row, the model is the degenerate pendulum case. With two rows, the model is the side view balancebot. With three rows, the model is the top-viewed balancebot, including orientation. With five rows, the model includes position in world frame.
    '''
    x1, x2, x3, x4 = states
    u = inputs

    x1_dot = x2
    x2_dot = (B.a_4 * sin(x1)
              - ((B.a_2 ** 2) / (2 * B.a_1)) * sin(2 * x1) * (x2 ** 2)
              - (1 + (B.a_2 / B.a_1) * cos(x1)) * u) / (B.a_3 - ((B.a_2 ** 2) / B.a_1) * (cos(x1) ** 2))
    x3_dot = x4
    x4_dot = (B.a_2 * (x2 ** 2) * sin(x1)
              + u
              - (B.a_2 * cos(x1) * x2_dot)) / B.a_1

    return np.array([x1_dot, x2_dot, x3_dot, x4_dot])


def double_pendulum_dynamics_simple(states, inputs, P):
    """
    Calculate the time derivatives of [states] subject to [inputs] for the simple double pendulum model with massless rods.

    Arguments:
        states = numpy.array([[theta_1,], 
                              [w_1,], 
                              [theta_2,],
                              [w_2,]])

            Must be numpy array of shape (4, 1-)

        inputs = numpy.array([[motor_1_1, motor_1_2, ..], 
                              [motor_2_1, motor_2_2, ..]])

            Must be numpy array of shape (2, 1-)

    Returns:
        dynamics = numpy.array([[theta_1,], 
                                [w_1,], 
                                [theta_2,],
                                [w_2,]])

            numpy array of same shape as [states]
    """
    x1, x2, x3, x4 = states
    u1, u2 = inputs
    
    x1 = wrap2pi(x1)
    x3 = wrap2pi(x3)
    
    t1 = u1 - (P.fv_1 * abs(x2) + P.fs_1) # TODO: check for signs with friction terms
    t2 = u2 - (P.fv_2 * abs(x4) + P.fs_2)
    
    d12 = P.m_2 * (P.l_c2**2 + P.l_1 * P.l_c2 * cos(x3)) + P.I_2
    d11 = d12 + (P.m_1 * P.l_c1**2) + (P.m_2 * P.l_1**2) + P.I_1
    d22 = P.d_22
    D = np.array([[d11, d12], [d12, d22]])
    det_inv = 1 / (d11 * d22 - d12 * d12)
    
    
    h = -P.m_2 * P.l_1 * P.l_c2 * sin(x3)
    c121 = h
    c211 = h
    c221 = h
    c112 = -h
    
    g1 = g * ((P.m_1 * P.l_c1 + P.m_2 * P.l_1) * cos(x1) + P.m_2 * P.l_c2 * cos(x1 + x3))
    g2 = g * P.m_2 * P.l_c2 * cos(x1 + x3)
    
    eq1 = t1 - (c121 + c211) * x2 * x4 - c221 * x4 * x4 - g1
    eq2 = t2 - c112 * x2 * x2 - g2

    x1_dot = x2
    x2_dot = det_inv * (d22 * eq1 - d12 * eq2)
    x3_dot = x4
    x4_dot = det_inv * (d11 * eq2 - d12 * eq1)

    return np.array([x1_dot, x2_dot, x3_dot, x4_dot])


def mobilebot_dynamics(states, inputs):
    '''
    Calculate the time derivatives of [states] subject to [inputs] for the mobilebot model.

    Arguments:
        states = numpy.array([[x_1, x_2, ..], 
                              [y_1, y_2, ..], 
                              [theta_1, theta_2, ..]])

            Must be numpy array of shape (1-3, 1-)

        inputs = numpy.array([[v_1, v_2, ..], 
                              [w_1, w_2, ..]])

            Must be numpy array of shape (2, 1-)

    Returns:
        dynamics = numpy.array([[x_1, x_2, ..], 
                                [y_1, y_2, ..], 
                                [theta_1, theta_2, ..]])

            numpy array of same shape as [states]
    '''
    x, y, theta = states
    v, w = inputs

    return np.array([v * cos(theta),
                     v * sin(theta),
                     w])


def mobilebot_differential(motor_inputs, mb_params):
    wheel_radius, drive_base = mb_params
    v_left, v_right = motor_inputs * wheel_radius

    v = (v_right + v_left) / 2
    w = (v_right - v_left) / drive_base

    return np.array([v, w])


def mobilebot_integrate(states, motor_inputs, mb_params, dt):
    inputs = mobilebot_differential(motor_inputs, mb_params)
    return integrate_dynamics(mobilebot_dynamics, states, inputs, dt)


def inverse_jacobian(reference, state, info):
    '''
    Control law that takes a reference state and current state and returns the invers jacobian control inputs to that referene.
    Args:
        reference = np.array([[x1], [x2], ..])
        state = np.array([[x1], [x2], ..])
        info = [jacobian_function, args, ]
    '''


def double_pend_info_ij(l_0, l_1):
    return [lambda q: np.array([[-l_0 * sin(q[0]) - l_1 * sin(q[0] + q[1]), -l_1 * sin(q[0] + q[1])],
                                [l_0 * cos(q[0]) + l_1 * cos(q[0] +
                                                           q[1]), l_1 * cos(q[0] + q[1])],
                                [0, 0],
                                [0, 0],
                                [0, 0],
                                [1, 1]])
            ]


def PID_control(reference, state, running_total, previous_error, dt, PID):
    error = reference - state
    output = PID.kp * (error) + PID.ki * (error + running_total) + \
        PID.kd * (error - previous_error) / dt
    return output, running_total + error, error


def queue_controller_thread(reference_queue, state_queue, control_queue, control_law, info, default_state, recurrent=False):
    '''
    Function that listens to a [state_queue], implements some control_law(reference, state, info, recurrent_info) -> inputs, recurrent_info, and puts inputs to a [control_queue].
    '''
    current_state = default_state
    while True:
        try:
            current_state = state_queue.get(block=False)
            current_reference = reference_queue.get(block=False)
        except:
            pass

        if recurrent:
            inputs, recurrent_info = control_law(
                current_reference, current_state, info, recurrent_info)
        else:
            inputs = control_law(current_reference, current_state, gains)

        try:
            control_queue.put(inputs, block=False)
        except:
            pass
