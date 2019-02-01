
import argparse
import gym
import gymfc
import matplotlib.pyplot as plt
import numpy as np
from mpi4py import MPI
import math
import os
import time
from gymfc.envs.gazebo_env import GazeboEnv

def eval(env, ac):
    """ Evaluate an environment with the given policy """
    ob = env.reset()
    while True:
        ob, reward, done, info = env.step(ac)
        if done:
            break
    env.close()

class Sim(GazeboEnv):
    def step(self, ac):
        self.step_sim(ac)
        done = False
        return self.state(), 0, done, {} 

    def state(self):
        return np.zeros(7)

    def sample_target(self):
        return np.zeros(3)


if __name__ == "__main__":

    parser = argparse.ArgumentParser("")
    parser.add_argument('config', help='')

    args = parser.parse_args()
    config_path = args.config
    print ("Loading config from ", config_path)
    os.environ["GYMFC_CONFIG"] = config_path


    env = Sim()
    env.render()
    eval(env, np.ones(4))
