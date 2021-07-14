from boa.snake_env import SnakeEnv


if __name__ == "__main__":
    game = SnakeEnv()
    print(game.action_space)
    for i in range(5):
        game.step(None)

    print(game._snake_robot.get_link_angles())
    print(game._snake_robot.get_joint_angles())
    print(game._snake_robot.get_joint_angvels())
    while True:
        game.step(None)
        game.render()
        if game.current_iteration % 100 == 0:
            game.reset()
