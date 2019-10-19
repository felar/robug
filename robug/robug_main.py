import rclpy
import tensorflow as tf
from rclpy.node import Node


class RobugNode(Node):

    def __init__(self):
        # Init the node, setting the name to robug_node
        super().__init__(node_name="robug_node")

        # Subscribers, publishers, etc. can/will get initialized here in the future.


def main():

    rclpy.init()

    # Initialize ROS2 node
    ros_node = RobugNode()

    # "spin" the node. This halts execution until rclpy gets shut down.
    rclpy.spin(node=ros_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
