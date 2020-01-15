import robottools as rt
import numpy as np
import pygame
from queue import Queue
from threading import Thread

# Immutable variables
K = np.array((0, 0, 0))
R = np.array((255, 0, 0))
G = np.array((0, 255, 0))
B = np.array((0, 0, 255))
W = np.array((255, 255, 255))
BACKGROUND_COLOR = W
TITLE = 'double_pendulum'
FPS = 60

# Class definitions
class DoublePendulum:
    def __init__(self, l0, l1):
        self.l0 = l0
        self.l1 = l1

# Function definitions


# Mutable variables
display_size = (640, 480)
done = False
reference_id = 0

# Instances
arm = DoublePendulum(1, 1)
reference_q = Queue()
state_q = Queue()
control_q = Queue()

t = Thread(target=rt.queue_controller_thread, args=(reference_q, state_q, control_q, rt.inverse_jacobian, rt.double_pend_info_ij(arm.l0, arm.l1), default_state))
# t.daemon = True
t.start()

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

        # Click parsing
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                reference_id += 1
            if event.button == 2:
                pass
            if event.button == 3:
                reference_id -= 1

    # Logic
    reference_id %= len(reference_trajectories)
    reference_trajectory = reference_trajectories[reference_id]

    # Drawing
    screen.fill(BACKGROUND_COLOR)
    text_string = 'Current Reference Trajectory: {}'.format(reference_id)
    pt = 36
    font = pygame.font.SysFont('sftransrobotics', pt)
    text_obj = font.render(text_string, True, K, W)
    text_rect = text_obj.get_rect(topleft=[0, 0])
    screen.blit(text_obj, text_rect)

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
quit()
