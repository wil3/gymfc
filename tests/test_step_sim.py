import argparse
import os
from gymfc.envs.fc_env import FlightControlEnv 
import numpy as np
import time

def step_sim(env, ac, delay=0):
    """ Evaluate an environment with the given policy """
    ob = env.reset()
    while True:
        ob = env.step_sim(ac)
        print ("F=", env.force)
        s = input("[enter] = Step, q = Quit")
        if s == 'q' or env.is_done():
            break
    env.close()

class Sim(FlightControlEnv):

    def __init__(self, aircraft_config, config_filepath=None, max_sim_time=1, verbose=False):
        super().__init__(aircraft_config, config_filepath=config_filepath, verbose=verbose)
        self.max_sim_time = max_sim_time

    def state(self):
        pass

    def desired_state(self):
        pass

    def is_done(self):
        return self.sim_time > self.max_sim_time

    def on_observation(self, ob):
        pass

    def on_reset(self):
        pass

if __name__ == "__main__":

    parser = argparse.ArgumentParser("Step the simulator with the given motor values.")
    parser.add_argument('aircraftconfig', help="File path of the aircraft SDF.")
    parser.add_argument('value', nargs='+', type=float, help="List of control signals, one for each motor.")
    parser.add_argument('--gymfc-config', default=None, help="Option to override default GymFC configuration location.")
    parser.add_argument('--delay', default=0, type=float, help="Second delay betwee steps for debugging purposes.")
    parser.add_argument('--max-sim-time', default=1, type=float, help="Time in seconds the sim should run for.")
    parser.add_argument('--verbose', action="store_true")


    args = parser.parse_args()

    env = Sim(args.aircraftconfig, config_filepath=args.gymfc_config, max_sim_time=args.max_sim_time, verbose=args.verbose)
    env.render()
    step_sim(env, np.array(args.value), delay=args.delay)


