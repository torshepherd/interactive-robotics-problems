import numpy as np
from queue import Queue
from threading import Thread


def project_screen_2d(world_points, R, p, scale):
    return np.matmul(R * scale, (world_points + p / scale)).astype(int)


def project_world_2d(screen_points, R, p, scale):
    return np.matmul(np.linalg.inv(R * scale), (screen_points)) - p / scale


def integrate_dynamics(dynamics, states, inputs, dt):
    return states + dt * dynamics(states, inputs)


def P2C_COLUMN_VECTOR(r, theta):
    return r * np.array([[np.cos(theta)], [np.sin(theta)]])


def balancebot_dynamics(states, inputs):
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

    if np.shape(states)[0] == 1:
        # Degenerate pendulum case, states = [theta]
        pass
    pass


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

    return np.array([v * np.cos(theta),
                     v * np.sin(theta),
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


def double_pend_info_ij(L0, L1):
    return [lambda q: np.array([[-L0 * np.sin(q[0]) - L1 * np.sin(q[0] + q[1]), -L1 * np.sin(q[0] + q[1])],
                                [L0 * np.cos(q[0]) + L1 * np.cos(q[0] +
                                                                 q[1]), L1 * np.cos(q[0] + q[1])],
                                [0, 0],
                                [0, 0],
                                [0, 0],
                                [1, 1]])
            ]


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
