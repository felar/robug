import rclpy
import tensorflow as tf

from tf_agents.agents.reinforce import reinforce_agent
from tf_agents.drivers import dynamic_episode_driver
from tf_agents.environments import tf_py_environment
from tf_agents.metrics import tf_metrics
from tf_agents.networks import actor_distribution_network
from tf_agents.replay_buffers import tf_uniform_replay_buffer

from .robug_env import RobugEnv

network_layers = (100, 50)
normalize_returns = True
learning_rate = 0.00001
discount = 0.5

bot_speed = 0.1
steering_speed = 0.8

iterations = 10
episodes_per_iteration = 5
replay_buffer_size = 3600

tensorboard_log_directory = './tensorboard_logs'


def main():

    enable_tensorboard_logging()

    print("[ROBUG] Initializing ROS and environment...")

    rclpy.init()

    py_environment = RobugEnv(bot_speed, steering_speed, discount)
    tf_env = tf_py_environment.TFPyEnvironment(py_environment)

    print("[ROBUG] Setting up neural net...")

    actor_net = actor_distribution_network.ActorDistributionNetwork(
        tf_env.observation_spec(),
        tf_env.action_spec(),
        fc_layer_params=network_layers)

    optimizer = tf.compat.v1.train.RMSPropOptimizer(learning_rate=learning_rate)

    agent = reinforce_agent.ReinforceAgent(
        tf_env.time_step_spec(),
        tf_env.action_spec(),
        actor_network=actor_net,
        optimizer=optimizer,
        normalize_returns=normalize_returns,
        debug_summaries=True,
        summarize_grads_and_vars=True
    )
    agent.initialize()

    collect_policy = agent.collect_policy

    replay_buffer = tf_uniform_replay_buffer.TFUniformReplayBuffer(
        data_spec=agent.collect_data_spec,
        batch_size=tf_env.batch_size,
        max_length=replay_buffer_size)

    collection_metrics = [
        tf_metrics.NumberOfEpisodes(),
        tf_metrics.EnvironmentSteps(),
        tf_metrics.AverageReturnMetric(),
        tf_metrics.AverageEpisodeLengthMetric()
    ]

    driver = dynamic_episode_driver.DynamicEpisodeDriver(
        tf_env,
        collect_policy,
        observers=[replay_buffer.add_batch] + collection_metrics,
        num_episodes=episodes_per_iteration
    )

    print("[ROBUG] Starting training...")

    for i in range(iterations):
        driver.run()
        experience = replay_buffer.gather_all()
        agent.train(experience)
        replay_buffer.clear()

    print("[ROBUG] Finished training, shutting down.")

    rclpy.shutdown()


def enable_tensorboard_logging():
    train_summary_writer = tf.summary.create_file_writer(tensorboard_log_directory)
    train_summary_writer.set_as_default()


if __name__ == '__main__':
    main()
