import argparse
import os
from gymfc.envs.fc_env import FlightControlEnv 
import numpy as np
import time

def step_sim(env, ac, delay=0):
    """ Evaluate an environment with the given policy """
    #ob = env.reset()
    while True:
        ob = env.step_sim(ac)

        if delay > 0:
            time.sleep(delay)
        if env.is_done():
            break
    env.close()

class Sim(FlightControlEnv):
    def state(self):
        pass

    def desired_state(self):
        pass

    def is_done(self):
        return self.sim_time > 1

    def on_observation(self):
        pass

    def on_reset(self):
        pass

if __name__ == "__main__":

    parser = argparse.ArgumentParser("Step the simulator with the given motor values.")
    parser.add_argument('config', help="Path to the GymFC configuration JSON file.")
    parser.add_argument('value', nargs='+', type=float, help="Control signals")
    parser.add_argument('--delay', type=float, help="Second delay betwee steps")


    args = parser.parse_args()
    config_path = args.config
    print ("Loading config from ", config_path)
    os.environ["GYMFC_CONFIG"] = config_path


    env = Sim()
    env.render()
    step_sim(env, np.array(args.value), delay=args.delay)
