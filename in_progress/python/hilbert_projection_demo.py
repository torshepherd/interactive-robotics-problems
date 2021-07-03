'''
S C Rn, x_0 in Rn
Is the set A:= {x in Rn: ||x-x_0||<=||x-y|| for all y in S} convex?

Visualize in R^2:
'''

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
TITLE = 'hilbert_projection_demo'
FPS = 60
SCALE = 500

# Class definitions


# Function definitions
def draw_samples(distribution):
    for row in distribution:
        pygame.draw.circle(screen, B, (SCALE*row[:2]).astype(int), 3, 0)

def draw_set(set_s):
    for y in set_s:
        pygame.draw.circle(screen, R, (SCALE*y[:2]).astype(int), 5, 0)
        
def draw_point(p):
    for x in p:
        pygame.draw.circle(screen, G, (SCALE*x[:2]).astype(int), 5, 0)

def recalculate_dist(dist, S, x_0):
    new_dist = []
    bool_vector = np.ones(np.shape(np.linalg.norm(dist,axis=1)))
    for y in S:
        bool_vector *= np.linalg.norm(dist - x_0, np.inf, axis=1) <= np.linalg.norm(dist - y, np.inf, axis=1)
    for i, row in enumerate(dist):
        if bool_vector[i]:
            new_dist.append(row)
            
    return np.array(new_dist)
                
def add_point(pt, dist, S, x_0):
    S = np.append(S, pt, axis=0)
    dist = recalculate_dist(dist, S, x_0)
    return dist, S


# Mutable variables
display_size = (SCALE, SCALE)
done = False
prev_time = time.time()


# Empty set S
S = np.random.random((1, 2))

# Random x_0 in R^2 within ((0, 1), (0, 1))
x_0 = np.random.random((1, 2))

dist = recalculate_dist(np.random.random((100000, 2)), S, x_0)


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
                # Mouse parsing
                mouse_pos = pygame.mouse.get_pos()
                x = mouse_pos[0]
                y = mouse_pos[1]
                
                point = np.array([[x, y]])/SCALE
                dist, S = add_point(point, dist, S, x_0)
    
    # Logic
    time_diff = time.time() - prev_time
    
    if time_diff > 100:
        point = [S[-1] + 0.05 * (np.random.random(2) - 0.5)]
        dist, S = add_point(point, dist, S, x_0)
        prev_time = time.time()
    
    # Drawing
    screen.fill(BACKGROUND_COLOR)
    draw_samples(dist)
    draw_set(S)
    draw_point(x_0)
    
    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
quit()
