import math
import numpy as np
import gym
from gym import spaces
from gym.utils import seeding
from gymfc.envs.fc_env import FlightControlEnv
import time
from neuroflight_trainer.gyms.rewards import RewardEnv

class RampEnv(RewardEnv): 
    def __init__(self,  
                 max_rate = 800, 
                 max_sim_time = 10,
                 aircraft_config=None,
                 action_bounds = [0, 1],
                 num_inputs = 6
                 ): 
        super().__init__(
            aircraft_config = aircraft_config,
            action_bounds = action_bounds,
            num_inputs = num_inputs,
            max_sim_time = max_sim_time
        )

        self.max_rate = max_rate
        self.step_width = max_sim_time/2.0
        self.rising = True

        self.ep_command_delta = np.zeros(3)
        self.first_step = True

        self.ts = [] #Track times
        self.es = [] #Track errors

    def step(self, action):
        # begin with a non-zero step
        if self.first_step:
            self.first_step = False
            self.ep_command_delta = self.sample_target()/1000.0
            self.omega_target = np.zeros(3)

        return super().step(action)

    def command_generator(self):
        if self.sim_time > self.step_width and self.rising: 
            self.ep_command_delta *= -1
            self.rising = False

        self.omega_target += self.ep_command_delta

    def reset(self):
        self.first_step = True
        self.rising = True
        self.ep_command_delta = np.zeros(3)
        self.omega_target = np.zeros(3)

        self.ts = [] #Track times
        self.es = [] #Track errors
        return super().reset()

    def sample_target(self):
        """ Sample a random angular velocity """
        if not self.np_random:
            seed = int(time.time()* 1e6) 
            self.seed(seed)
        return self.np_random.randint(-self.max_rate, self.max_rate, size=3)


class RampErrDeltaErrEnv(RampEnv):
    def __init__(self,  
                 max_rate = 800,
                 max_sim_time = 2,
                 aircraft_config=None,
                 action_bounds = [-1, 1],
                 num_inputs = 6
                 ): 

        super().__init__(
                 max_sim_time = max_sim_time,
                 aircraft_config=aircraft_config,
                 max_rate = max_rate,
                 action_bounds = action_bounds,
                num_inputs = num_inputs
        )

    def state(self):
        """Get the current state"""
        err_rad = np.array(list(map(math.radians, self.measured_error)))
        last_err_rad = np.array(list(map(math.radians, self.last_measured_error)))

        error_delta = (err_rad - last_err_rad)
        return np.concatenate([err_rad, error_delta]) 
