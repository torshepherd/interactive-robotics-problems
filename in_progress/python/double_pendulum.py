import robottools as rt
import numpy as np
from time import time
from robottools import PID_control, integrate_dynamics
from numpy import pi
import pygame

# Immutable variables
K = np.array((0, 0, 0))
R = np.array((255, 0, 0))
G = np.array((0, 255, 0))
B = np.array((0, 0, 255))
W = np.array((255, 255, 255))

BACKGROUND_COLOR = W
DISPLAY_SIZE = (640, 480)
TITLE = 'double_pendulum'
FPS = 60
SCALE = 250


# Function definitions
def reset_states():
    return np.array([[0],
                     [0],
                     [0],
                     [0]])
    
def draw_pendulum(P, states):
    x1 = states[0, 0]
    x2 = states[1, 0]
    x3 = states[2, 0]
    x4 = states[3, 0]

    o = np.array([DISPLAY_SIZE[0] / 2, 200])
    draw_l1 = (o + P.l_1 * SCALE * np.array([np.sin(x1), np.cos(x1)])).astype(int)
    draw_l2 = (draw_l1 + P.l_2 * SCALE * np.array([np.sin(x1 + x3), np.cos(x1 + x3)])).astype(int)

    pygame.draw.aaline(screen, K, o, draw_l1)
    pygame.draw.aaline(screen, K, draw_l1, draw_l2)

    pygame.draw.circle(screen, W, draw_l1, 5, 0)
    pygame.draw.circle(screen, K, draw_l1, 5, 1)
    
    pygame.draw.circle(screen, W, draw_l2, 5, 0)
    pygame.draw.circle(screen, K, draw_l2, 5, 1)


# Mutable variables
done = False
states = reset_states()

# Instances
P = rt.DoublePendulum(0.5, 0.5, 0, 0, 0.25, 0.25, 0, 0, 0, 0)

# Initialization
pygame.init()
screen = pygame.display.set_mode(DISPLAY_SIZE, 0)
pygame.display.set_caption(TITLE)
icon_surf = pygame.image.load('logo_small.png')
pygame.display.set_icon(icon_surf)
clock = pygame.time.Clock()
font = pygame.font.SysFont('lucidasansregular', 36)
prev_time = time()

# Main display loop
while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

        # User input
        # WASD parsing
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                states = reset_states()

    # Logic
    # Compute time difference
    time_diff = time() - prev_time
    prev_time = time()
    if time_diff == 0:
        time_diff = 0.0001

    inputs = np.array([[0],
                       [0]])

    states = integrate_dynamics(rt.double_pendulum_dynamics_simple,
                                states,
                                inputs,
                                time_diff,
                                P)

    # Drawing
    screen.fill(BACKGROUND_COLOR)
    draw_pendulum(P, states)

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
quit()
