import random
import pygame
import pymunk
import pymunk.pygame_util
import gym
import numpy as np
from .snake_robot import SnakeRobot


class SnakeEnv(gym.Env):
    OBSTACLE_DISTANCE_AVG = 60
    OBSTACLE_DISTANCE_VAR = 15
    SCREEN_WIDTH = 1200
    SCREEN_HEIGHT = 600
    DELTA_T = 1 / 30
    N_LINKS = 10

    def __init__(self):
        super().__init__()

        self._space = pymunk.Space()
        self._obstacles: [pymunk.Shape] = []
        self._snake_robot: SnakeRobot = None
        self._reset_obstacles()
        self._reset_robot()

        # Graphics are set to none and are initialized when `render` is called
        # for the first time
        self._screen = None
        self._draw_options = None
        self._clock = None

        self.current_iteration = 0
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(self.N_LINKS,), dtype=np.float)

    def _reset_obstacles(self):
        if self._obstacles:
            self._space.remove(*self._obstacles)
            self._obstacles = []

        for x in range(0, self.SCREEN_WIDTH, self.OBSTACLE_DISTANCE_AVG):
            for y in range(0, self.SCREEN_HEIGHT, self.OBSTACLE_DISTANCE_AVG):
                rand_x = random.uniform(-self.OBSTACLE_DISTANCE_VAR, self.OBSTACLE_DISTANCE_VAR)
                rand_y = random.uniform(-self.OBSTACLE_DISTANCE_VAR, self.OBSTACLE_DISTANCE_VAR)
                offset = (x + rand_x, y + rand_y)
                obstacle = pymunk.Circle(self._space.static_body, radius=random.uniform(5, 10), offset=offset)
                obstacle.friction = 0
                obstacle.elasticity = 0.5
                self._obstacles.append(obstacle)

        self._space.add(*self._obstacles)

    def _reset_robot(self):
        if self._snake_robot:
            self._snake_robot.remove_from_space(self._space)
            self._snake_robot = None

        self._snake_robot = SnakeRobot(
            n_links=self.N_LINKS,
            link_mass=10,
            link_length=30,
            radius=10,
            position=(10, 300)
        )
        self._snake_robot.add_to_space(self._space)

    def step(self, action):
        err_msg = f"{action} ({type(action)}) invalid)"
        assert self.action_space.contains(action), err_msg

        self._snake_robot.set_motor_speeds(action)
        self._space.step(self.DELTA_T)
        self.current_iteration += 1

        reward = self._snake_robot.get_head_x_vel()
        done = self.current_iteration > 300

        return None, reward, done, {}

    def reset(self):
        self._reset_obstacles()
        self._reset_robot()
        self.current_iteration = 0

    def render(self, mode='human'):

        if self._screen is None:
            pygame.init()
            self._screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
            self._draw_options = pymunk.pygame_util.DrawOptions(self._screen)
            self._clock = pygame.time.Clock()

        self._screen.fill(pygame.Color("white"))
        self._space.debug_draw(self._draw_options)
        pygame.display.flip()
        # Delay fixed time between frames
        self._clock.tick(30)
        pygame.display.set_caption(f"fps: {self._clock.get_fps()}")


