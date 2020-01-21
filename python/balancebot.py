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
    x1, x2, x3, x4 = states
    scaling = 10
    pygame.draw.aaline(screen, K, [0,400], [640,400])
    draw_center = np.array(np.array([320, 400]) + scaling * np.array([int(B.R_w * x3), -int(B.R_w)]))
    
    pygame.draw.circle(screen, K, draw_center, int(B.R_w * scaling), 1)
    print((draw_center + scaling * B.L * np.array([np.sin(x1), -np.cos(x1)])).astype(int))
    pygame.draw.aaline(screen, K, draw_center, (draw_center + scaling * B.L * np.array([np.sin(x1), -np.cos(x1)])).astype(int))

# Mutable variables
display_size = (640, 480)
done = False
fps = 0
prev_time = time.time()
states = np.array([[np.pi],
                   [0],
                   [0],
                   [0]])
inputs = 0

# Instances
B = rt.Balancebot(1, 1, 1, 1, 1, 0.25)

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
                inputs = 1
            if event.key == pygame.K_d:
                inputs = -1
        
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_a:
                inputs = 0
            if event.key == pygame.K_d:
                inputs = 0
        
    # Logic
    # Compute time differencing
    time_diff = time.time() - prev_time
    prev_time = time.time()
    
    # March states
    states = rt.integrate_dynamics(rt.balancebot_dynamics, states, inputs, time_diff, B)
    
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
    print(states)
    
    draw_balancebot(B, states)
    
    pygame.display.flip()

pygame.quit()
quit()
