import math

import numpy as np

from boa.snake_env import SnakeEnv


if __name__ == "__main__":
    game = SnakeEnv()
    for i in range(100):
        game.reset()
        done = False
        total_reward = 0.0
        while not done:
            action = game.action_space.sample()
            observation, reward, done, _info = game.step(action)
            total_reward += reward
            print(observation)
            print(observation.shape)
            game.render()
            #print(f"measurement: {observation}")
            #print(game._snake_robot.get_joint_angles())
        print(f"Run {i}: Total reward {total_reward}")