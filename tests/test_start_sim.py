import argparse

from gymfc.envs.fc_env import FlightControlEnv


class Sim(FlightControlEnv):
    def __init__(self, aircraft_config, config=None, verbose=False):
        super().__init__(aircraft_config, config_filepath=config, verbose=verbose)

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
    parser.add_argument('aircraftconfig', help="File path of the aircraft SDF.")
    parser.add_argument('--gymfc-config', help="Option to override default GymFC configuration location.")
    parser.add_argument('--verbose', action="store_true")

    args = parser.parse_args()

    env = Sim(args.aircraftconfig, args.gymfc_config, args.verbose)
    env.render().wait()

