import numpy as np
import pygame
import robottools as rt
import time

# Immutable variables
K = np.array((0, 0, 0))
R = np.array((255, 0, 0))
G = np.array((0, 255, 0))
B = np.array((0, 0, 255))
W = np.array((255, 255, 255))
BACKGROUND_COLOR = W
TITLE = 'leg_vis_nocontrol'
FPS = 60

DT = 0.25  # When to move to next point
R = np.array([[1, 0], [0, -1]])
P = np.array([[320], [-140]])
SCALE = 500

points = np.loadtxt('leg_0.traj', delimiter=',')
num_points = np.shape(points)[0]


class Leg:
    def __init__(self, l_1, l_2):
        self.theta_1 = 0
        self.theta_2 = 0
        self.l_1 = l_1
        self.l_2 = l_2

# Function definitions


def draw_leg(leg):
    points = []
    points[0] = np.array([[0], [0]])
    points[1] = points[0] + rt.polar2cart(leg.l_1, leg.theta_1)
    points[2] = points[1] + rt.polar2cart(leg.l_2, leg.theta_2)

    pygame.draw.aalines(screen, K, False, [rt.project_screen_2d(
        np.transpose(pt), R, P, SCALE) for pt in points])
    
    return points


def convert_point(world_point):
    return np.array([world_point[1:]])


# Mutable variables
display_size = (640, 480)
done = False
ind = 0
state = 1
prev_time = time.time()

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
        # Click parsing
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                state += 1
                state = state % 2

    # Logic
    if not state:
        time_diff = time.time() - prev_time
        if time_diff > DT:
            ind += 1
            ind = ind % num_points
            prev_time = time.time()

    point = points[ind]
    arm_points = [np.array([[0, 0]]), convert_point(point)]
    arm_points_screen = [rt.project_screen_2d(
        np.transpose(pt), R, P, SCALE) for pt in arm_points]

    # Drawing
    screen.fill(BACKGROUND_COLOR)
    pygame.draw.aalines(screen, B, False, arm_points_screen)

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
quit()
