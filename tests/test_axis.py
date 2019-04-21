import argparse
import os
from gymfc.envs.fc_env import FlightControlEnv 
import numpy as np
import time

def step_sim(env, delay=0, scale=1):
    motor_commands = [
        np.array([1, 1, 0, 0]), # roll left
        np.array([0, 0, 1, 1]), # roll right
        np.array([1, 0, 1, 0]), # pitch forward 
        np.array([0, 1, 0, 1]), # pitch backwards 
        np.array([1, 0, 0, 1]), # Yaw CCW 
        np.array([0, 1, 1, 0])  # Yaw CW
    ]
    motor_command_strings = ["Roll Left", 
                             "Roll Right",
                             "Pitch Forward", 
                             "Pitch Backwards", 
                             "Yaw CCW",
                             "Yaw CW"]
    running = True
    while len(motor_commands) > 0 and running:

        m = motor_commands.pop(0)
        s = motor_command_strings.pop(0)
        print (s)
        print ("-----------------------------------")
        ob = env.reset()
        while True:
            ob = env.step_sim(m * scale)
            print ("Angular Velocity=", env.imu_angular_velocity_rpy)
            s = input("[enter] = Step, n = Next Command, q = Quit")
            if s == 'n' or env.is_done():
                break
            elif s == 'q':
                running = False
                break
    env.close()

class Sim(FlightControlEnv):

    def __init__(self, aircraft_config, config_filepath=None, max_sim_time=1):
        super().__init__(aircraft_config, config_filepath=config_filepath, verbose=False)
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

    parser = argparse.ArgumentParser("")
    parser.add_argument('aircraftconfig', help="File path of the aircraft SDF.")
    parser.add_argument('--gymfc-config', default=None, help="Option to override default GymFC configuration location.")
    parser.add_argument('--delay', default=0, type=float, help="Second delay betwee steps for debugging purposes.")
    parser.add_argument('--scale', default=1, type=float, help="Scale for the motor.")
    parser.add_argument('--max-sim-time', default=1, type=float, help="Time in seconds the sim should run for.")


    args = parser.parse_args()

    env = Sim(args.aircraftconfig, config_filepath=args.gymfc_config, max_sim_time=args.max_sim_time)
    env.render()
    step_sim(env, delay=args.delay, scale=args.scale)


