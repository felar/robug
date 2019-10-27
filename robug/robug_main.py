import rclpy

from .robug_env import RobugEnv


def main():

    rclpy.init()

    environment = RobugEnv()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
