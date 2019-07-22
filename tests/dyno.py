import argparse
import os
from gymfc.envs.fc_env import FlightControlEnv 
import numpy as np
import time
import matplotlib.pyplot as plt

def step_input(env, output_file, max_command, max_sim_time, input_type="step"):
    """
        Args:
            env
            output_file
            max_command
    """
    ob = env.reset()
    command = max_command
    data = []

    peak_time = max_sim_time/2.0 
    ramp_steps = peak_time/env.stepsize
    command_step = 1/ramp_steps
    while True:
        ob = env.step_sim(command)
        data.append([env.sim_time,command[0], env.esc_motor_angular_velocity, env.esc_force, env.esc_torque ])
        if input_type == "step":
            if env.sim_time > peak_time:
                command = np.zeros(1)
        else:
            if env.sim_time < peak_time:
                command += command_step
            else:
                command -= command_step


        if env.sim_time > max_sim_time:
            break

    env.close()
    np.savetxt(output_file, data, header="time,command,velocity,force,torque", delimiter=",")

class Sim(FlightControlEnv):

    def __init__(self, aircraft_config, config_filepath=None, verbose=False):
        super().__init__(aircraft_config, config_filepath=config_filepath, verbose=verbose)

if __name__ == "__main__":

    parser = argparse.ArgumentParser("A dyno emulator for validating motor models in simulation.")
    parser.add_argument('output', help="Output file.")
    parser.add_argument('aircraftconfig', help="File path of the aircraft SDF.")
    parser.add_argument('value', nargs='+', type=float, help="Throttle value for the motor.")
    parser.add_argument('--gymfc-config', default=None, help="Option to override default GymFC configuration location.")
    parser.add_argument('--delay', default=0, type=float, help="Second delay betwee steps for debugging purposes.")
    parser.add_argument('--max-sim-time', default=1, type=float, help="Time in seconds the sim should run for.")
    parser.add_argument('--verbose', action="store_true", help="Provide additional logs from Gazebo.")
    parser.add_argument('--render', action="store_true", help="Display the Gazebo client.")
    parser.add_argument('--input-type', default="step", help="step or ramp")


    args = parser.parse_args()

    env = Sim(args.aircraftconfig, config_filepath=args.gymfc_config, verbose=args.verbose)
    if args.render:
        env.render()
    step_input(env, args.output, np.array(args.value), args.max_sim_time, args.input_type)


