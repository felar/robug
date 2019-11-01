import rclpy
from tensorflow.python.keras import optimizers
from tf_agents.agents.reinforce import reinforce_agent
from tf_agents.drivers import dynamic_episode_driver
from tf_agents.environments import tf_py_environment
from tf_agents.networks import actor_distribution_network
from tf_agents.replay_buffers import tf_uniform_replay_buffer

from .robug_env import RobugEnv


def main():

    rclpy.init()

    py_environment = RobugEnv()
    tf_env = tf_py_environment.TFPyEnvironment(py_environment)

    actor_net = actor_distribution_network.ActorDistributionNetwork(
        tf_env.observation_spec(),
        tf_env.action_spec(),
        fc_layer_params=(100, 50))

    optimizer = optimizers.Adam()

    agent = reinforce_agent.ReinforceAgent(
        tf_env.time_step_spec(),
        tf_env.action_spec(),
        actor_network=actor_net,
        optimizer=optimizer,
        normalize_returns=True)
    agent.initialize()

    collect_policy = agent.collect_policy

    replay_buffer = tf_uniform_replay_buffer.TFUniformReplayBuffer(
        data_spec=agent.collect_data_spec,
        batch_size=tf_env.batch_size,
        max_length=3600)

    driver = dynamic_episode_driver.DynamicEpisodeDriver(
        tf_env,
        collect_policy,
        observers=[replay_buffer.add_batch],
        num_episodes=5
    )

    for _ in range(10):
        driver.run()
        experience = replay_buffer.gather_all()
        agent.train(experience)
        replay_buffer.clear()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
