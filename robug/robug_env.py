from threading import Thread

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

    def __init__(self, bot_speed, discount):
        super().__init__()

        self.bot_speed = bot_speed
        self.discount = discount

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

        # TF Agents doesn't automatically call reset() when the robot drives against a wall.
        # Instead, it just keeps calling step() to get more data. Because of this, we use
        # _episode_ended to "mark" when the simulation should be reset in the next step
        self._episode_ended = False

        # This spins the node in parallel so we can continue running Tensorflow while the node is spinning
        Thread(target=self.spin_node).start()

    def spin_node(self):
        rclpy.spin(self.ros_node)

    def update_observation(self, msg):
        distances = np.array(msg.ranges, dtype=np.dtype('float64'))

        # The turtlebot sometimes returns infinity as a value, which Tensorflow can't handle. So we cap the
        # "view distance" at 3.5
        distances[distances > 3.5] = 3.5

        # Remove "NaN" (Not a Number) values because TF can't handle them
        for index in range(len(distances)):
            if np.isnan(distances[index]):
                distances[index] = 0

        self.latest_observation = distances

    def action_spec(self):
        return array_spec.BoundedArraySpec(
          shape=(),
          dtype=np.dtype('int32'),
          minimum=0,
          maximum=2,
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

        # Send the request to reset the simulation
        self.reset_client.call(reset_request)

        # Reset the observation - otherwise, the bot will lose immediately because the old observation
        # (where we drove against the wall) is still there
        self.latest_observation = np.array([0.2] * 360, dtype=np.dtype('float64'))

        self._episode_ended = False

        return time_step.restart(self.latest_observation)

    def _step(self, action):

        if self._episode_ended:
            # The last action ended the episode. Ignore the current action and start
            # a new episode.
            return self.reset()

        # Go left
        if action == 0:
            vel_turn = Twist()
            vel_turn.linear.x = self.bot_speed
            vel_turn.angular.z = self.bot_speed
            self.velocity_publisher.publish(vel_turn)

        # Go straight
        elif action == 1:
            vel_turn = Twist()
            vel_turn.linear.x = self.bot_speed
            vel_turn.angular.z = 0.0
            self.velocity_publisher.publish(vel_turn)

        # Go right
        elif action == 2:
            vel_turn = Twist()
            vel_turn.linear.x = self.bot_speed
            vel_turn.angular.z = -self.bot_speed
            self.velocity_publisher.publish(vel_turn)

        else:
            raise ValueError('`action` should be -1, 0 or 1.')

        current_observation = self.latest_observation

        # The sensor itself is not in the middle of the TB3, so we need to check for
        # walls in a larger range
        # min(...) finds the smallest number in the array
        in_front_of_wall = (0.13 >= min(current_observation))

        if in_front_of_wall:
            reward = -100
        else:
            reward = 1

        if in_front_of_wall:
            self._episode_ended = True
            return time_step.termination(current_observation, reward)
        else:
            return time_step.transition(current_observation, reward, discount=self.discount)
