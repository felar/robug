import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf_agents.environments import py_environment
from tf_agents.specs import array_spec
from tf_agents.trajectories import time_step


class RobugEnv(py_environment.PyEnvironment):

    def __init__(self):
        super().__init__()

        # Create a node and a client for the reset_simulation service that we can use later to reset the simulation
        self.ros_node = rclpy.create_node('minimal_node')

        self.reset_client = self.ros_node.create_client(Empty, 'reset_simulation')

        self.velocity_publisher = self.ros_node.create_publisher(msg_type=Twist, topic='cmd_vel')

        # We have to use qos_profile=qos_profile_sensor_data here, otherwise we don't receive input.
        # See https://answers.ros.org/question/334649/ros2-turtlebot3-gazebo-scan-topic-subscriber-is-not-calling-the-callback-function/
        self.laser_scan_subscription = self.ros_node.create_subscription(
            msg_type=LaserScan,
            topic='scan',
            callback=self.update_observation,
            qos_profile=qos_profile_sensor_data
        )
        # Start with 0.2 everywhere as observation because if we set it to zero the bot would immediately lose
        self.latest_observation = np.array([0.2] * 360, dtype=np.dtype('float64'))

    def update_observation(self, msg):
        distances = np.array(msg.ranges, dtype=np.dtype('float64'))

        # The turtlebot sometimes returns infinity as a value, which Tensorflow can't handle. So we cap the
        # "view distance" at 3.5
        distances[distances > 3.5] = 3.5
        self.latest_observation = distances

    def action_spec(self):
        return array_spec.BoundedArraySpec(
          shape=(),
          dtype=np.dtype('int32'),
          minimum=-1,
          maximum=1,
          name='action'
        )

    def observation_spec(self):
        return array_spec.BoundedArraySpec(
            shape=(360,),
            dtype=np.dtype('float64'),
            name='observation'
        )

    def _reset(self):
        # Create an empty request to reset the simulation (emtpy because we don't specify any arguments for the request)
        reset_request = Empty.Request()

        # Wait for the reset service to appear
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.ros_node.get_logger().info('simulation reset service not available, waiting...')

        # Get a 'future' object that can tell use when the request is completed
        future = self.reset_client.call_async(reset_request)

        # Wait until the request is completed
        rclpy.spin_until_future_complete(self.ros_node, future)

        return time_step.restart(self.latest_observation)

    def _step(self, action):

        # Go left
        if action == -1:
            vel_turn = Twist()
            vel_turn.linear.x = 0.1
            vel_turn.angular.z = 0.1
            self.velocity_publisher.publish(vel_turn)

        # Go straight
        elif action == 0:
            vel_turn = Twist()
            vel_turn.linear.x = 0.1
            vel_turn.angular.z = 0.0
            self.velocity_publisher.publish(vel_turn)

        # Go right
        elif action == 1:
            vel_turn = Twist()
            vel_turn.linear.x = 0.1
            vel_turn.angular.z = -0.1
            self.velocity_publisher.publish(vel_turn)

        else:
            raise ValueError('`action` should be -1, 0 or 1.')

        current_observation = self.latest_observation

        in_front_of_wall = (0.0 == min(current_observation))

        if in_front_of_wall:
            reward = -100
        else:
            reward = 1

        if in_front_of_wall:
            return time_step.termination(current_observation, reward)
        else:
            return time_step.transition(current_observation, reward)
