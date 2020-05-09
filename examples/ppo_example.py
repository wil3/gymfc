"""Train a neuro-flight controller attitude control using the PPO RL
algorithm.

It is recommended before training for an extended period of time to set the ckpt_freq
and timestep arguments to small numbers and test everything is functioning as
expected. 

The Tensorflow input tensor is named 'pi/ob:0', and the output tensor is named
'pi/pol/final/BiasAdd:0'. This can be used to extract the subgraph and evaluate
the graph in pb format.
"""
from gymfc_nf.envs import *
import os.path
import time
import datetime
import subprocess
import numpy as np
import gym
import argparse
import tensorflow as tf

tf.logging.set_verbosity(tf.logging.ERROR)
def get_commit_hash():
    out = subprocess.run(["git", "describe", "--always"], stdout=subprocess.PIPE, encoding="utf-8")
    commit = out.stdout.strip()
    return commit

def get_training_name():
    now = datetime.datetime.now()
    timestamp = now.strftime('%y%m%d-%H%M%S')
    return "baselines_" + get_commit_hash() + "_" + timestamp


def train(env, num_timesteps, seed, flight_log_dir=None, ckpt_dir=None, 
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
    flight_log = None
    #if flight_log_dir:
    #    flight_log = FlightLog(flight_log_dir)
    if render:
        env.render()
    env.seed(workerseed)
    set_global_seeds(workerseed)
    pposgd_simple.learn(env, policy_fn,
            max_timesteps=num_timesteps,
            timesteps_per_actorbatch=horizon,
            clip_param=0.2, entcoeff=0.0,
            optim_epochs=optim_epochs, optim_stepsize=optim_stepsize, optim_batchsize=optim_batchsize,
            gamma=0.99, lam=0.95, schedule=schedule,
            flight_log = flight_log,
            ckpt_dir = ckpt_dir,
            restore_dir = restore_dir,
            save_timestep_period= ckpt_freq
            )
    env.close()



if __name__ == '__main__':
    parser = argparse.ArgumentParser("Synthesize a neuro-flight controller.")
    parser.add_argument('--checkpoint_dir', default="../../checkpoints")
    parser.add_argument('--twin', default="./gymfc_nf/twins/nf1/model.sdf",
                        help="File path of the aircraft digitial twin/model SDF.")
    parser.add_argument('--gym-id', default="gymfc_nf-step-v1")
    parser.add_argument('--timesteps', type=int, default=10e6)
    parser.add_argument('--ckpt-freq', type=int, default=100e3)
    args = parser.parse_args()

    training_dir = os.path.join(args.checkpoint_dir, get_training_name())
    print ("Storing results to ", training_dir)

    seed = np.random.randint(0, 1e6) 
    ckpt_dir = os.path.join(training_dir, "checkpoints")
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

    train(env, timesteps, seed, flight_log_dir=None, ckpt_dir = ckpt_dir,
        render = render,
        ckpt_freq = ckpt_freq, schedule = schedule, optim_stepsize = step_size, 
        horizon = horizon, optim_batchsize = batchsize, optim_epochs = epochs,
        gamma = gamma)

