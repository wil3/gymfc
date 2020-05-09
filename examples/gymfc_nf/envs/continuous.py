import math
import numpy as np
import gym
from gym import spaces
from gym.utils import seeding
from gymfc.envs.fc_env import FlightControlEnv
import time
from neuroflight_trainer.gyms.rewards import RewardEnv

class ContinuousEnv(RewardEnv): 
    """An environment that continuously generates setpoints for the agent to 
    track until a max sim time is reached. 

    This is to simulate a more realistic environment in which the next 
    maneuver will begin at random orientations. However it is more difficult 
    to compare agents using this environment than the StepEnv, and it was 
    also show the StepEnv was sufficient in training the agent rate control.
    """
    def __init__(self,  max_rate = 800, pulse_width = 2, max_sim_time = 60,
                 aircraft_config=None, action_bounds = [-1, 1], state_fn = None,
                 version = 0): 
        super().__init__(
            aircraft_config = aircraft_config,
            action_bounds = action_bounds,
            max_sim_time = max_sim_time,
             state_fn = state_fn,
            version = version)
        self.max_rate = max_rate
        self.pulse_width = pulse_width
        self.first_step = True
        self.next_pulse_time = pulse_width

        self.step_max_y = np.zeros(4)
        self.step_min_y = np.ones(4) 
        self.in_band_step_counter = 0
        self.last_command_y = np.zeros(4)
        self.command_did_change = False
        self.angular_rate_sp = np.zeros(3)


    def setpoint_generator(self):
        if self.sim_time > self.next_pulse_time:
            self.command_start_time = self.sim_time

            self.command_counter = 0
            self.command_did_change = False
            self.last_command_y = self.y.copy()

            self.angular_rate_sp = self.sample_target()
            self.next_pulse_time += self.pulse_width
            self.target_reached = np.array([False, False, False])
        else:
            self.command_counter += 1

    def step(self, action):
        # begin with a non-zero step
        if self.first_step:
            self.first_step = False
            self.angular_rate_sp = self.sample_target()
        return super().step(action)

    def reset(self):
        self.first_step = True
        self.next_pulse_time = self.pulse_width
        self.angular_rate_sp = np.zeros(3)
        self.last_command_y = np.zeros(4)
        self.command_did_change = False
        self.command_counter = 0
        return super().reset()

    def sample_target(self):
        """ Sample a random angular velocity """
        if not self.np_random:
            seed = int(time.time()* 1e6) 
            self.seed(seed)
        return self.np_random.randint(-self.max_rate, self.max_rate, size=3)


