import numpy as np
import rclpy
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

        # TODO: Return the new observation after resetting, etc.

        return 0

    def _step(self, action):
        # TODO: Execute the action using a ROS publisher, get the sensor input from ROS, calculate rewards
        #  and return them as a time_step with observation and reward for this step

        return 0
