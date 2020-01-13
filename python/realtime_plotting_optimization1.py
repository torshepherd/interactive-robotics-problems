import numpy as np
from scipy.optimize import minimize
from scipy.integrate import odeint
from scipy.interpolate import interp1d
import pygame
import matplotlib.pyplot as plt
import robottools as rt

R = [255,   0,   0]
G = [0, 255,   0]
B = [0,   0, 255]
K = [0,   0,   0]
W = [255, 255, 255]

pygame.init()
size = np.array((800, 800))
screen = pygame.display.set_mode(size)

clock = pygame.time.Clock()

z_0 = np.array([0, 0, 0])
z_f = np.array([1, 0.1, 0])
n = 10
T_SPAN = np.linspace(0, 2, n)
DT = T_SPAN[1] - T_SPAN[0]


def dynamics(z, t, u_func):
    x, y, theta = z
    v, w = u_func(t)

    return np.array([
        v * np.cos(theta),
        v * np.sin(theta),
        w])


def dynamics_i(z, i, u_array):
    x, y, theta = z
    v, w = u_array[i]

    return np.array([
        v * np.cos(theta),
        v * np.sin(theta),
        w])


def J(u):
    pass


def integrate_control_input_45(input_array, z_0):
    '''
    Pure function to integrate the robot dynamics using odeint from scipy.integrate.

    Arguments:
        input_array
        z_0

    Returns:
        trajectory of states
    '''
    u_array = np.reshape(input_array, [2, int(np.shape(input_array)[0] / 2)])
    v = interp1d(T_SPAN, u_array[0], kind='cubic', fill_value="extrapolate")
    w = interp1d(T_SPAN, u_array[1], kind='cubic', fill_value="extrapolate")
    def u_func(t): return [v(t), w(t)]

    return odeint(dynamics, z_0, T_SPAN, args=(u_func,))


def integrate_control_input_1(input_array, z_0):
    sol = np.zeros((n, 3))
    sol[0, :] = z_0
    u_array = np.reshape(input_array, [2, int(np.shape(input_array)[0] / 2)])
    u_array = np.transpose(u_array)
    for i in range(1, n):
        sol[i, :] = integrate_step(u_array[i-1], sol[i-1, :], DT)
        #sol[i, :] = DT * dynamics_i(sol[i - 1, :], i - 1, u_array) + sol[i-1,:]

    return sol


def integrate_step(input_0, z_0, dt):
    u_array = np.append([input_0], [[0, 0]], axis=0)
    return z_0 + dt * dynamics_i(z_0, 0, u_array)


def cost_fun_45(input_array, z_0, z_f):
    sol = integrate_control_input_45(input_array, z_0)

    error = z_f - sol
    weight = np.array([1, 1, 0])
    return sum(np.linalg.norm(error * error * weight, axis=1))


def cost_fun_1(input_array, z_0, z_f):
    sol = integrate_control_input_1(input_array, z_0)

    error = z_f - sol
    weight = np.array([1, 1, 0])
    return sum(np.linalg.norm(error * error * weight, axis=1))


def get_path(z_0, z_f, method='integrate'):
    u_0 = np.array([2 for blank in range(n)] + [0.1 for blank in range(n)])
    b_0 = [(0, 2) for blank in range(n)] + [(-2, 2) for blank in range(n)]
    if method == 'integrate':
        result = minimize(cost_fun_45, u_0, args=(z_0, z_f,),
                          bounds=b_0, options={'disp': True})
    elif method == 'euler_equality':
        result = minimize(cost_fun_1, u_0, args=(z_0, z_f,),
                          bounds=b_0, options={'disp': True})
    print(result.x)
    return integrate_control_input_1(result.x, z_0)


def project_world(screen_pts):
    return (screen_pts - np.array([400, 400])) * np.array([.01, -.01])


def project_screen(world_pts):
    return ((world_pts + np.array([4, -4])) * np.array([100, -100])).astype(int)


def draw_arrow(pt):
    tip = pt[:2] + 0.1 * np.array([np.cos(pt[-1]), np.sin(pt[-1])])
    corner_1 = pt[:2] + 0.05 * np.array([-np.sin(pt[-1]), np.cos(pt[-1])])
    corner_2 = pt[:2] + 0.05 * np.array([np.sin(pt[-1]), -np.cos(pt[-1])])
    to_draw = [project_screen(corner_1).tolist(), project_screen(
        tip).tolist(), project_screen(corner_2).tolist()]
    pygame.draw.aalines(screen, B, True, to_draw)


screen.fill(W)

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

        if event.type == pygame.MOUSEBUTTONDOWN:
            screen.fill(W)
            z_f = np.append(project_world(pygame.mouse.get_pos()), 0)
            print(z_f)
            sol = get_path(z_0, z_f, method='euler_equality')
            pts = project_screen(sol[:, :2])
            pygame.draw.aalines(screen, K, False, pts.tolist())
            pt = sol[0]
            draw_arrow(pt)
            z_0 = sol[1]

    pygame.display.flip()
    clock.tick(60)


'''
plt.plot(sol[:, 0], sol[:, 1])
plt.xlim([-1,1])
plt.ylim([-1,1])
plt.show()

test_inputs = 2 * np.array([[1], [1]])
u_array = np.tile(test_inputs, [1, 100])

v = interp1d(T_SPAN, u_array[0], kind='cubic', fill_value="extrapolate")
w = interp1d(T_SPAN, u_array[1], kind='cubic', fill_value="extrapolate")
u_func = lambda t: [v(t), w(t)]

sol = odeint(dynamics, z_0, T_SPAN, args=(u_func,))

plt.plot(sol[:, 0], sol[:, 1])
plt.show()
'''
