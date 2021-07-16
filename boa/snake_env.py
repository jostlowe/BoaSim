import math
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
    N_JOINTS = N_LINKS - 1

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

        # 1 is max CW speed, -1 = max CCW speed
        self.action_space = gym.spaces.Box(
            low=-1,
            high=1,
            shape=(self.N_JOINTS,),
            dtype=np.float
        )

        self.observation_space = gym.spaces.Box(
            low=-math.pi / 2,
            high=math.pi / 2,
            shape=(self.N_LINKS*3, ),
            dtype=np.float
        )

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
                obstacle.elasticity = 0.2
                obstacle.collision_type = 0
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

    def _is_done(self):
        head_x, head_y = self._snake_robot.head.body.position
        is_timeout = self.current_iteration > 300
        is_at_end = head_x > self.SCREEN_WIDTH
        is_out_of_bounds = head_y > self.SCREEN_HEIGHT or head_y < 0

        return is_timeout or is_at_end or is_out_of_bounds

    def _get_observation(self):
        forces = np.concatenate(self._snake_robot.get_collision_forces()) / 1000
        angles = np.array(self._snake_robot.get_link_angles())
        return np.concatenate((forces, angles))

    def step(self, action):
        # Set the joint motor speeds
        self._snake_robot.set_motor_speeds(action)

        # Iterate the physics simulation
        self._space.step(self.DELTA_T)

        # Calculate the reward as the global x-velocity of the head link
        (head_xvel, _head_yvel) = self._snake_robot.head.body.velocity
        reward = head_xvel

        # Check if the episode is done
        done = self._is_done()

        self.current_iteration += 1

        observation = self._get_observation()

        return observation, reward, done, {}

    def reset(self):
        self._reset_obstacles()
        self._reset_robot()

        # Do some iterations to stabilize the simulation
        for i in range(10):
            self.step([0 for _ in self._snake_robot._joints])

        self.current_iteration = 0

        return self._get_observation()

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
