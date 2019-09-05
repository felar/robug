import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class RobugNode(Node):

    def __init__(self):
        # Init the node, setting the name to robug_node
        super().__init__(node_name="robug_node")

        # Subscribers, publishers, etc. can/will get initialized here in the future.
        self.subscription = self.create_subscription(msg_type=LaserScan, topic='scan', callback=self.sub_call)

    def sub_call (self, msg):
        print('Nachricht empfangen!')
        print(msg)



def main():

    rclpy.init()

    # Initialize ROS2 node
    ros_node = RobugNode()

    # "spin" the node. This halts execution until rclpy gets shut down.
    rclpy.spin(node=ros_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
