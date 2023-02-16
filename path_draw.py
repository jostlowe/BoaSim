import math
import random
import sys

import pygame

import pymunk
import pymunk.pygame_util
from math import pi
from itertools import accumulate

from itertools import pairwise
random.seed(1)

L = 15
R = 5
N = 15
Nj = N-1
M = 0.4
x0, y0 = 100, 200

path_deg = [0]*Nj + [ 51, 0, 0, -90, -88, 0, 0, 90, 90, 0, 0, -55, 0, 0, -45, -90, -45, 0, 0, 90] + 50*[0]
path = [deg/180*pi for deg in path_deg]
print(path)

obstacle_pos = [(200, 215), (300, 185), (400, 215), (520, 160), (580, 150), (570, 220), (640, 190), (680, 155), (720, 190), (640, 240)]

def main():
    pygame.init()
    screen = pygame.display.set_mode((1000, 1000))
    clock = pygame.time.Clock()
    space = pymunk.Space()
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    target_angles = [0]*(N-1)
    t = 0
    tp = 0

    for pos in obstacle_pos:
        space.add(pymunk.Circle(space.static_body, 10, pos))

    path_g = list(accumulate(path))
    path_pos = [(x0, y0)]
    for i in range(len(path)):
        x_prev, y_prev = path_pos[i]
        angle = path_g[i]
        x = x_prev + 2*L*math.cos(angle)
        y = y_prev - 2*L*math.sin(angle)
        path_pos.append((x,y))

    for pos_a, pos_b in pairwise(path_pos):
        print(pos_a, pos_b)
        space.add(pymunk.Segment(space.static_body, pos_a, pos_b, 5))

    while True:
        t += 1/200
        tp = t/2
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                sys.exit(0)

        space.step(1 / 200.0)
        screen.fill((255, 255, 255))

        space.debug_draw(draw_options)

        pygame.display.flip()
        clock.tick(200)


if __name__ == "__main__":
    main()
