import robottools as rt
import numpy as np
import pygame
from time import time
from robottools import PID_control, integrate_dynamics
from numpy import pi

# Immutable variables
K = np.array((0, 0, 0))
R = np.array((255, 0, 0))
G = np.array((0, 255, 0))
B = np.array((0, 0, 255))
W = np.array((255, 255, 255))

BACKGROUND_COLOR = W
DISPLAY_SIZE = np.array((640, 480))
TITLE = 'balancebot'
SCALE = 500

FALL_LIMIT = (2 * pi / 3)
REFERENCE_LIMIT = 1 * pi

# Function definitions


def draw_balancebot(B, states):
    x1 = states[0, 0]
    x2 = states[1, 0]
    x3 = states[2, 0]
    x4 = states[3, 0]

    o = np.array([DISPLAY_SIZE[0] / 2, 400])
    draw_center = (o + SCALE * np.array([B.R_w * x3, -B.R_w])).astype(int)
    draw_com = (draw_center + SCALE * B.L *
                np.array([np.sin(x1), -np.cos(x1)])).astype(int)
    draw_wheel = (draw_center + SCALE * B.R_w *
                  np.array([np.sin(x3), -np.cos(x3)])).astype(int)

    pygame.draw.aaline(screen, K, [0, 400], [640, 400])
    pygame.draw.aaline(screen, K, draw_center, draw_com)

    pygame.draw.circle(screen, W, draw_center, int(B.R_w * SCALE), 0)
    pygame.draw.circle(screen, K, draw_center, int(B.R_w * SCALE), 1)
    pygame.draw.aaline(screen, K, draw_center, draw_wheel)


def draw_ticker(B, x3_ref):
    o = np.array([DISPLAY_SIZE[0] / 2, 400])
    draw_center = (o + SCALE * np.array([B.R_w * x3_ref, 0])).astype(int)
    draw_endpt = (draw_center + np.array([0, 10]))

    pygame.draw.aaline(screen, K, draw_center, draw_endpt)


# Mutable variables
done = False
standing = True

# Instances
# mass_beam, mass_wheel, inertia_beam, inertia_wheel, length_center_of_mass, radius_wheel
B = rt.Balancebot(1, 0.06, 0.006, 0.000048, .1, 0.04)

# Kp, Kd, Ki
D1 = rt.PID(-2.5, -.2, -.1)
D2 = rt.PID(.02, 0.000, 0.003)

# Initialization
pygame.init()
screen = pygame.display.set_mode(DISPLAY_SIZE, 0)
pygame.display.set_caption(TITLE)
icon_surf = pygame.image.load('logo_small.png')
pygame.display.set_icon(icon_surf)
clock = pygame.time.Clock()
font = pygame.font.SysFont('lucidasansregular', 36)

# Main display loop
while not done:
    fps = 0
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
    clicked = 0
    standing = True
    while standing:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
                standing = False

            # Click parsing
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    clicked += 1
                    clicked %= 2

        # Mouse parsing
        if clicked == 1:
            x = pygame.mouse.get_pos()[0]
            x3_ref = ((x - 320) / (SCALE * B.R_w))

        # Compute time difference
        time_diff = time() - prev_time
        prev_time = time()
        if time_diff == 0:
            time_diff = 0.001

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

        # March states
        states = integrate_dynamics(rt.balancebot_dynamics,
                                    states,
                                    inputs,
                                    time_diff,
                                    B)

        # Check standing
        if abs(states[0, 0]) > FALL_LIMIT:
            states[1, 0] = 0
            standing = False

        fps = int(1 / time_diff)
        text_string = 'FPS: ' + str(fps)

        # Drawing
        screen.fill(BACKGROUND_COLOR)

        text_obj = font.render(text_string, True, K, W)
        text_rect = text_obj.get_rect(topleft=[0, 0])
        screen.blit(text_obj, text_rect)

        draw_balancebot(B, states)
        draw_ticker(B, x3_ref)

        pygame.display.flip()
        clock.tick(200)

pygame.quit()
quit()
