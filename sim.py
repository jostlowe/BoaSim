"""A L shape attached with a joint and constrained to not tip over.
This example is also used in the Get Started Tutorial. 
"""

__docformat__ = "reStructuredText"

import math
import random
import sys

import pygame

import pymunk
import pymunk.pygame_util
from math import pi

from itertools import pairwise
random.seed(1)

L = 15
R = 5
N = 15
Nj = N-1
M = 0.4
x0, y0 = 100, 200

path_deg = [0]*Nj + [ 51, 0, 0, -90, -88, 0, 0, 90, 90, 0, 0, -55, 0, 0, -45, -90, -45, 0, 0, 45, 45] + 50*[0]
path = [deg/180*pi for deg in path_deg]
#path = [0]*Nj + [ pi/4, 0, 0, -pi/2, -pi/2, 0, 0, pi/2, pi/2, 0, 0, -pi/4] +
print(path)

obstacle_pos = [(200, 215), (300, 185), (400, 215), (500, 185), (580, 150), (570, 220), (640, 190), (680, 155), (720, 190), (640, 240)]



class SnakeRobot():

    def __init__(self, space):
        self.bodies = [pymunk.Body() for _ in range(N)]
        for i, body in enumerate(self.bodies):
            body.position = (x0 + 2 * L * i, y0)

        self.segments = [pymunk.Segment(body, (-L, 0), (L, 0), R) for body in self.bodies]
        for segment in self.segments:
            segment.mass = M
            segment.friction = 0.2

        self.joints = [
            pymunk.PivotJoint(link_a, link_b, (L, 0), (-L, 0))
            for link_a, link_b in pairwise(self.bodies)
        ]
        for joint in self.joints:
            joint.collide_bodies = False

        self.motors = [
            pymunk.SimpleMotor(link_a, link_b, rate=0)
            for link_a, link_b in pairwise(self.bodies)
        ]
        for motor in self.motors:
            motor.max_force = 20000

        space.add(*self.bodies, *self.segments, *self.joints, *self.motors)

    def set_speeds(self, speeds):
        for motor, speed in zip(self.motors, speeds):
            motor.rate = speed

    @property
    def link_angles(self):
        return [body.angle for body in self.bodies]

    @property
    def joint_angles(self):
        return [a - b for a, b in pairwise(self.link_angles)]


def main():
    pygame.init()
    screen = pygame.display.set_mode((1000, 600))
    clock = pygame.time.Clock()
    space = pymunk.Space()
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    boa = SnakeRobot(space)
    target_angles = [0]*(N-1)
    t = 0
    tp = 0

    for pos in obstacle_pos:
        obstacle = pymunk.Circle(space.static_body, 10, pos)
        obstacle.friction = 0.2
        space.add(obstacle)

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

        s = math.ceil(tp)
        tl = tp-s+1

        #target_angles = path[s: s+Nj]
        target_angles = [ksi_a*(1-tl) + ksi_b*(tl) for ksi_a, ksi_b in pairwise(path[s:s+N])]

        target_speeds = [
            5 * (target - actual)
            for target, actual in zip(target_angles, boa.joint_angles)
        ]

        boa.set_speeds(target_speeds)
        space.debug_draw(draw_options)

        pygame.display.flip()
        clock.tick(200)


if __name__ == "__main__":
    main()
