import robottools as rt
import numpy as np
import pygame
from queue import Queue
import time

# Immutable variables
K = np.array((0, 0, 0))
R = np.array((255, 0, 0))
G = np.array((0, 255, 0))
B = np.array((0, 0, 255))
W = np.array((255, 255, 255))
BACKGROUND_COLOR = W
TITLE = 'mobilebot'
FPS = 60

# Class definitions


class Car:
    def __init__(self, x, y, theta):
        self.states = np.array([[x], [y], [theta]])
        self.params = np.array([[1], [2.0]])

# Function definitions


def draw_car(surface, car, screen_R, screen_p, scale):
    tip = car.states[:2] + 1 * \
        np.array([np.cos(car.states[-1]), np.sin(car.states[-1])])
    corner_1 = car.states[:2] + 0.5 * car.params[1] * \
        np.array([-np.sin(car.states[-1]), np.cos(car.states[-1])])
    corner_2 = car.states[:2] + 0.5 * car.params[1] * \
        np.array([np.sin(car.states[-1]), -np.cos(car.states[-1])])
    list_of_points = [project_screen_2d(corner_1, screen_R, screen_p, scale).T.tolist()[0], project_screen_2d(
        tip, screen_R, screen_p, scale).T.tolist()[0], project_screen_2d(corner_2, screen_R, screen_p, scale).T.tolist()[0]]
    pygame.draw.aalines(surface, B, True, list_of_points)


def project_screen_2d(world_points, R, p, scale):
    return np.matmul(R * scale, (world_points + p / scale)).astype(int)


def project_world_2d(screen_points, R, p, scale):
    return np.matmul(np.linalg.inv(R * scale), (screen_points / scale)) - p


def integrate_car(car, motor_inputs, dt):
    inputs = rt.mobilebot_differential(motor_inputs, car.params)
    return rt.integrate_dynamics(rt.mobilebot_dynamics, car.states, inputs, dt)


# Mutable variables
display_size = (640, 480)
done = False
scale = 50
screen_R = np.array([[1, 0], [0, -1]])
screen_p = np.array([[320], [-240]])
motor_left = 0
motor_right = 0
prev_time = time.time()

# Instances
car = Car(0, 0, 0)

# Initialization
pygame.init()
screen = pygame.display.set_mode(display_size, pygame.RESIZABLE)
pygame.display.set_caption(TITLE)
clock = pygame.time.Clock()

# Main display loop
while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

        # User input
        # Resizable window logic
        if event.type == pygame.VIDEORESIZE:
            display_size = event.size
            pygame.display.set_mode(display_size, pygame.RESIZABLE)

        # WASD parsing
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                motor_left += 1
                motor_right += 1
            if event.key == pygame.K_a:
                motor_left += 1
            if event.key == pygame.K_s:
                motor_left -= 1
                motor_right -= 1
            if event.key == pygame.K_d:
                motor_right += 1

        if event.type == pygame.KEYUP:
            if event.key == pygame.K_w:
                motor_left -= 1
                motor_right -= 1
            if event.key == pygame.K_a:
                motor_left -= 1
            if event.key == pygame.K_s:
                motor_left += 1
                motor_right += 1
            if event.key == pygame.K_d:
                motor_right -= 1

        # Click parsing
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                scale *= 1.5
            if event.button == 2:
                pass
            if event.button == 3:
                scale /= 1.5

    # Logic
    time_diff = time.time() - prev_time
    prev_time = time.time()

    car.states = integrate_car(car, np.array(
        [[motor_left], [motor_right]]), time_diff)

    # Drawing
    screen.fill(BACKGROUND_COLOR)
    text_string = 'Car Position: {}, {}'.format(
        car.states[0].round(2), car.states[1].round(2))
    pt = 24
    font = pygame.font.SysFont('lucidasansregular', pt)
    text_obj = font.render(text_string, True, K, W)
    text_rect = text_obj.get_rect(topleft=[0, 0])
    screen.blit(text_obj, text_rect)

    draw_car(screen, car, screen_R, screen_p, scale)

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
quit()
