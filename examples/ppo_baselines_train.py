"""Train a neuro-flight controller attitude control using the PPO RL
algorithm.

It is recommended before training for an extended period of time to set the 
ckpt_freq and timestep arguments to small numbers and test everything is 
functioning as expected.

The Tensorflow input tensor is named 'pi/ob:0', and the output tensor is named
'pi/pol/final/BiasAdd:0'. This can be used to extract the subgraph and evaluate
the graph in pb format which is later needed by Neuroflight. An example of this
in Python is in gymfc_nf.policies.PpoBaselinesPolicy

"""
from gymfc_nf.envs import *
import os.path
import time
import datetime
import subprocess
import numpy as np
np.seterr('ignore')
import gym
import argparse
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import tensorflow as tf

tf.logging.set_verbosity(tf.logging.ERROR)
def get_commit_hash():
    out = subprocess.run(["git", "describe", "--always"], stdout=subprocess.PIPE, encoding="utf-8")
    commit = out.stdout.strip()
    return commit

def get_training_name():
    now = datetime.datetime.now()
    timestamp = now.strftime('%Y%m%d-%H%M%S')
    return "baselines_" + get_commit_hash() + "_" + timestamp


def train(env, num_timesteps, seed, ckpt_dir=None,
          render=False, ckpt_freq=0, restore_dir=None, optim_stepsize=3e-4,
          schedule="linear", gamma=0.99, optim_epochs=10, optim_batchsize=64,
          horizon=2048):

    from baselines.common.fc_learning_utils import FlightLog
    from mpi4py import MPI
    from baselines import logger
    from baselines.ppo1.mlp_policy import MlpPolicy
    from baselines.common import set_global_seeds
    from baselines.ppo1 import pposgd_simple
    import baselines.common.tf_util as U
    sess = U.single_threaded_session()
    sess.__enter__()

    rank = MPI.COMM_WORLD.Get_rank()
    if rank == 0:
        logger.configure()
    else:
        logger.configure(format_strs=[])
    logger.set_level(logger.DISABLED)
    workerseed = seed + 1000000 * rank
    def policy_fn(name, ob_space, ac_space):
        return MlpPolicy(name=name, ob_space=ob_space, ac_space=ac_space,
                         hid_size=32, num_hid_layers=2)
    if render:
        env.render()
    env.seed(workerseed)
    set_global_seeds(workerseed)
    pposgd_simple.learn(env, policy_fn,
                        max_timesteps=num_timesteps,
                        timesteps_per_actorbatch=horizon,
                        clip_param=0.2, entcoeff=0.0,
                        optim_epochs=optim_epochs,
                        optim_stepsize=optim_stepsize,
                        optim_batchsize=optim_batchsize,
                        gamma=0.99, lam=0.95, schedule=schedule,
                        flight_log = None,
                        ckpt_dir = ckpt_dir,
                        restore_dir = restore_dir,
                        save_timestep_period= ckpt_freq
                        )
    env.close()


class StepCallback:

    def __init__(self, total_timesteps, log_freq=1):
        """
        Args:
            total_timesteps: Total timesteps for training
            log_freq: Number of episodes until an update log message is printed
        """
        self.timesteps = total_timesteps
        self.steps_taken = 0
        self.es = []
        self.sps = []
        self.ep = 1
        self.rewards = []
        self.log_freq = log_freq
        self.log_header = ["Ep",
                           "Done",
                           "Steps",

                           "r",
                           "-ydelta",
                           "+ymin",
                           "+/-e",
                           "-ahigh",
                           "-nothing",

                           "score",

                           "pMAE",
                           "qMAE",
                           "rMAE"]

        header_format = ["{:<5}",
                         "{:<7}",
                         "{:<15}",

                         "{:<15}",
                         "{:<15}",
                         "{:<15}",
                         "{:<15}",
                         "{:<15}",
                         "{:<15}",

                         "{:<10}",

                         "{:<7}",
                         "{:<7}",
                         "{:<7}"]
        self.header_format = "".join(header_format)

        log_format_entries = ["{:<5}",
                              "{:<7.0%}",
                              "{:<15}",

                              "{:<15.0f}",
                              "{:<15.0f}",
                              "{:<15.0f}",
                              "{:<15.0f}",
                              "{:<15.0f}",
                              "{:<15.0f}",

                              "{:<10.2f}",

                              "{:<7.0f}",
                              "{:<7.0f}",
                              "{:<7.0f}"]

        self.log_format = "".join(log_format_entries)

    def callback(self, local, state, reward, done):

        self.es.append(local.true_error)
        self.sps.append(local.angular_rate_sp)

        assert local.ind_rewards[0] <= 0 # oscillation penalty
        assert local.ind_rewards[1] >= 0 # min output reward
        assert local.ind_rewards[3] <= 0 # over saturation penalty
        assert local.ind_rewards[4] <= 0 # do nothing penalty

        self.rewards.append(local.ind_rewards)

        if done:
            if self.ep == 1:
                print(self.header_format.format(*self.log_header))
            # XXX (wfk) Try this new score, we need something normalized to handle the
            # random setpoints. Scale by the setpoint, larger setpoints incur
            # more error. +1 prevents divide by zero
            mae = np.mean(np.abs(self.es))
            mae_pqr = np.mean(np.abs(self.es), axis=0)
            e_score = mae / (1 + np.mean(np.abs(self.sps)))
            self.steps_taken += local.step_counter

            if self.ep % self.log_freq == 0:
                ave_ind_rewards = np.mean(self.rewards, axis=0)
                ind_rewards = ""
                for r in ave_ind_rewards:
                    ind_rewards += "{:<15.2f} ".format(r)

                log_data = [
                    self.ep,
                    self.steps_taken/self.timesteps,
                    self.steps_taken,

                    np.mean(self.rewards),
                    ave_ind_rewards[0],
                    ave_ind_rewards[1],
                    ave_ind_rewards[2],
                    ave_ind_rewards[3],
                    ave_ind_rewards[4],

                    e_score,
                    mae_pqr[0],
                    mae_pqr[1],
                    mae_pqr[2]
                ]
                print (self.log_format.format(*log_data))

            self.ep += 1
            self.es = []
            self.sps = []
            self.rewards = []

if __name__ == '__main__':
    parser = argparse.ArgumentParser("Synthesize a neuro-flight controller.")
    parser.add_argument('--model_dir', default="../../models",
                        help="Directory where models are saved to.")
    parser.add_argument('--twin', default="./gymfc_nf/twins/nf1/model.sdf",
                        help="File path of the aircraft digitial twin/model SDF.")
    parser.add_argument('--seed', type=int, default=np.random.randint(0, 1e6),
                        help="Seed for RNG.")
    parser.add_argument('--gym-id', default="gymfc_nf-step-v1")
    parser.add_argument('--timesteps', type=int, default=10e6)
    parser.add_argument('--ckpt-freq', type=int, default=100e3)
    args = parser.parse_args()

    training_dir = os.path.join(args.model_dir, get_training_name())

    seed = args.seed
    ckpt_dir = os.path.abspath(os.path.join(training_dir, "checkpoints"))
    print ("Saving checkpoints to {}.".format(ckpt_dir))
    render = False
    # How many timesteps until a checkpoint is saved
    ckpt_freq = args.ckpt_freq

    # RL hyperparameters
    timesteps = args.timesteps
    schedule = "linear"
    step_size = 1e-4
    horizon = 512
    batchsize = 64
    epochs = 5
    gamma = 0.99

    env_id = args.gym_id
    env = gym.make(env_id)
    def sample_noise(inst):
        # Experiementally derived for MatekF7 FC, see Chapter 5 of "Flight
        # Controller Synthesis Via Deep Reinforcement Learning" for methodology.
        r_noise = inst.np_random.normal(-0.25465, 1.3373)
        p_noise = inst.np_random.normal(0.241961, 0.9990)
        y_noise = inst.np_random.normal(0.07906, 1.45168)
        return np.array([r_noise, p_noise, y_noise])

    env.sample_noise = sample_noise
    env.set_aircraft_model(args.twin)

    cb = StepCallback(timesteps)
    env.step_callback = cb.callback

    train(env, timesteps, seed, ckpt_dir = ckpt_dir, render = render,
          ckpt_freq = ckpt_freq, schedule = schedule, optim_stepsize = step_size,
          horizon = horizon, optim_batchsize = batchsize, optim_epochs = epochs,
          gamma = gamma)

