from boa.snake_env import SnakeEnv


if __name__ == "__main__":
    game = SnakeEnv()
    print(game.action_space)
    print(game._snake_robot.get_link_angles())
    print(game._snake_robot.get_joint_angles())
    count = 0
    for i in range(100):
        game.reset()
        done = False
        total_reward = 0.0
        while not done:
            action = game.action_space.sample()
            _observation, reward, done, _info = game.step(action)
            total_reward += reward
            game.render()
        print(f"Run {i}: Total reward {total_reward}")