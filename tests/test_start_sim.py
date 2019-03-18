import argparse
import math
import os
import time
from gymfc.envs.fc_env import FlightControlEnv 


class StartSim(FlightControlEnv):
    def state(self):
        pass
    def desired_state(self):
        pass
    def is_done(self):
        pass
    def on_observation(self):
        pass
    def on_reset(self):
        pass

if __name__ == "__main__":

    parser = argparse.ArgumentParser("Start the simulator so it can be inspected.")
    parser.add_argument('config', help="Path to the GymFC configuration JSON file.")

    args = parser.parse_args()
    config_path = args.config
    print ("Loading config from ", config_path)
    os.environ["GYMFC_CONFIG"] = config_path


    env = StartSim()
    env.render()

