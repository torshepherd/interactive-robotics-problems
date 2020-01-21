import robottools as rt
import numpy as np
import pygame
import time

# Immutable variables
K = np.array((0, 0, 0))
R = np.array((255, 0, 0))
G = np.array((0, 255, 0))
B = np.array((0, 0, 255))
W = np.array((255, 255, 255))
BACKGROUND_COLOR = W
TITLE = 'balancebot'
FPS = 60

# Class definitions


# Function definitions
def draw_balancebot(B, states):
    x1 = states[0, 0]
    x2 = states[1, 0]
    x3 = states[2, 0]
    x4 = states[3, 0]

    scaling = 100
    o = np.array([320, 400])
    draw_center = np.array(
        o + scaling * np.array([B.R_w * x3, -B.R_w])).astype(int)
    draw_com = np.array(draw_center + scaling * B.L *
                        np.array([np.sin(x1), -np.cos(x1)])).astype(int)
    draw_wheel = np.array(draw_center + scaling * B.R_w *
                          np.array([np.sin(x3), -np.cos(x3)])).astype(int)

    pygame.draw.aaline(screen, K, [0, 400], [640, 400])
    pygame.draw.aaline(screen, K, draw_center, draw_com)

    pygame.draw.circle(screen, W, draw_center, int(B.R_w * scaling), 0)
    pygame.draw.circle(screen, K, draw_center, int(B.R_w * scaling), 1)
    pygame.draw.aaline(screen, K, draw_center, draw_wheel)


# Mutable variables
display_size = [640, 480]
done = False
fps = 0
prev_time = time.time()
states = np.array([[0.05],
                   [0],
                   [0],
                   [0]])
inputs = 0
integrated_e_1 = 0
previous_e_1 = 0
integrated_e_2 = 0
previous_e_2 = 0
x3_ref = 0

# Instances
# mass_beam, mass_wheel, inertia_beam, inertia_wheel, length_center_of_mass, radius_wheel
B = rt.Balancebot(10, 0.6, 0.06, 0.00048, 1, 0.4)
D1 = rt.PID(-80, -20, -100)
D2 = rt.PID(.1, 0.000, 0.0001)

# Initialization
pygame.init()
screen = pygame.display.set_mode(display_size, 0)
pygame.display.set_caption(TITLE)
clock = pygame.time.Clock()

# Main display loop
while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

        # User input
        # WASD parsing
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_a:
                x3_ref += 1
            if event.key == pygame.K_d:
                x3_ref += -1

        if event.type == pygame.KEYUP:
            if event.key == pygame.K_a:
                pass
            if event.key == pygame.K_d:
                pass

    # Logic
    # Compute time differencing
    time_diff = time.time() - prev_time
    prev_time = time.time()

    # Run PID control
    x1_ref, integrated_e_2, previous_e_2 = rt.PID_control(x3_ref,
                                                          states[2, 0],
                                                          integrated_e_2,
                                                          previous_e_2,
                                                          time_diff,
                                                          D2)
    
    inputs, integrated_e_1, previous_e_1 = rt.PID_control(x1_ref,
                                                          states[0, 0],
                                                          integrated_e_1,
                                                          previous_e_1,
                                                          time_diff,
                                                          D1)

    # March states
    states = rt.integrate_dynamics(rt.balancebot_dynamics,
                                   states,
                                   inputs,
                                   time_diff,
                                   B)

    # Wrap by 2pi
    if abs(states[0, 0]) > (2 * np.pi / 3):
        states[1, 0] = 0
    states[0, 0] = (states[0, 0] + np.pi) % (np.pi * 2) - np.pi

    # Drawing
    screen.fill(BACKGROUND_COLOR)
    if time_diff != 0:
        fps = int(1 / time_diff)

    text_string = str(fps)
    pt = 36
    font = pygame.font.SysFont('lucidasansregular', pt)
    text_obj = font.render(text_string, True, K, W)
    text_rect = text_obj.get_rect(topleft=[0, 0])
    screen.blit(text_obj, text_rect)
    print(x1_ref)

    draw_balancebot(B, states)

    pygame.display.flip()

pygame.quit()
quit()
