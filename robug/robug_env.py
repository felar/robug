import numpy as np
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
          shape=(1,), dtype=np.int32, minimum=0, name='observation')

    def _reset(self):
        return 0

    def _step(self, action):
        return 0
