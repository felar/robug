import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf_agents.specs import array_spec
from tf_agents.environments import py_environment
from tf_agents.trajectories import time_step


class RobugEnv(py_environment.PyEnvironment):

    def __init__(self):
        super().__init__()

        # Create a node and a client for the reset_simulation service that we can use later to reset the simulation
        self.reset_node = rclpy.create_node('minimal_client')
        self.reset_client = self.reset_node.create_client(Empty, 'reset_simulation')
        self.velocity_publisher = self.create_publisher(msg_type=Twist, topic='cmd_vel')
        self.laser_scan_subscription = self.create_subscription(msg_type=LaserScan, topic='scan', callback=self.update_observation)
        self.latest_observation = [0.0] * 360

    def update_observation(self, msg):
        self.latest_observation = msg.ranges

    def action_spec(self):
        return array_spec.BoundedArraySpec(
          shape=(),
          dtype=np.int32,
          minimum=-1,
          maximum=1,
          name='action'
        )

    def observation_spec(self):
        return array_spec.BoundedArraySpec(
          shape=(1, 360), dtype=float, name='observation')

    def _reset(self):
        # Create an empty request to reset the simulation (emtpy because we don't specify any arguments for the request)
        reset_request = Empty.Request()

        # Wait for the reset service to appear
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.reset_node.get_logger().info('simulation reset service not available, waiting...')

        # Get a 'future' object that can tell use when the request is completed
        future = self.reset_client.call_async(reset_request)

        # Wait until the request is completed
        rclpy.spin_until_future_complete(self.reset_node, future)

        return time_step.restart(self.latest_observation)

    def _step(self, action):

        if action == -1:
            vel_turn = Twist()
            vel_turn.linear.x = 0.1
            vel_turn.angular.z = 0.1
            self.velocity_publisher.publish(vel_turn)
        elif action == 0:
            vel_turn = Twist()
            vel_turn.linear.x = 0.1
            vel_turn.angular.z = 0.0
            self.velocity_publisher.publish(vel_turn)
        elif action == 1:
            vel_turn = Twist()
            vel_turn.linear.x = 0.1
            vel_turn.angular.z = -0.1
            self.velocity_publisher.publish(vel_turn)
        else:
            raise ValueError('`action` should be -1, 0 or 1.')

        current_observation = self.latest_observation

        in_front_of_wall = False
        for distance in current_observation:
            if distance == 0.0:
                in_front_of_wall = True

        if in_front_of_wall:
            reward = -100
        else:
            reward = 1

        if in_front_of_wall:
            return time_step.termination(current_observation, reward)
        else:
            return time_step.transition(current_observation, reward)
