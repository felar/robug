import rclpy
import tensorflow as tf
import tf_agents as tfa
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class RobugNode(Node):

    def __init__(self):
        # Init the node, setting the name to robug_node
        super().__init__(node_name="robug_node")

        # Subscribers, publishers, etc. can/will get initialized here in the future.
        self.subscription = self.create_subscription(msg_type=LaserScan, topic='scan', callback=self.sub_call)
        self.publisher = self.create_publisher(msg_type=Twist, topic='cmd_vel')
        self.switch = True
        self.timer = self.create_timer(5, self.timer_callback)

    def sub_call (self, msg):
        vel_cmd = Twist()
        # vel_cmd.linear.x = 0.15
        # vel_cmd.angular.z = 0.1
        # self.publisher.publish(vel_cmd)

    def timer_callback(self):

        if self.switch == True:
            vel_straight = Twist()
            vel_straight.linear.x = 0.1
            vel_straight.angular.z = 0.0
            self.publisher.publish(vel_straight)
            self.switch = False
        else:
            vel_turn = Twist()
            vel_turn.linear.x = 0.0
            vel_turn.angular.z = 0.1
            self.publisher.publish(vel_turn)
            self.switch = True


def main():

    rclpy.init()

    # Initialize ROS2 node
    ros_node = RobugNode()

    # "spin" the node. This halts execution until rclpy gets shut down.
    rclpy.spin(node=ros_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
