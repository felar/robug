import numpy as np
import rclpy
from std_srvs.srv import Empty
from tf_agents.specs import array_spec
from tf_agents.environments import py_environment
from tf_agents.trajectories import time_step


class RobugEnv(py_environment.PyEnvironment):


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
        return 0

    def _step(self, action):
        return 0
