
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


class Sim(GazeboEnv):
    def step(self):
        pass

    def state(self):
        return np.ones(7)

    def sample_target(self):
        pass


if __name__ == "__main__":

    parser = argparse.ArgumentParser("Evaluate a PID controller")
    parser.add_argument('config', help='')

    args = parser.parse_args()
    config_path = args.config
    print ("Loading config from ", config_path)
    os.environ["GYMFC_CONFIG"] = config_path


    env = Sim()
    env.render()
