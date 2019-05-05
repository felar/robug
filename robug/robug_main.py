import rclpy
from rclpy.node import Node


class RobugNode(Node):

    def __init__(self):
        # Init the node, setting the name to robug_node
        super().__init__("robug_node")

        # Subscribers, publishers, etc. can/will get initialized here in the future.


def main(args=None):

    rclpy.init(args=args)

    # Initialize ROS2 node
    ros_node = RobugNode()

    # "spin" the node. This halts execution until rospy gets shut down.
    rclpy.spin(ros_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
