import math
import numpy as np
import gym
from gym import spaces
from gym.utils import seeding
from gymfc.envs.fc_env import FlightControlEnv
import time
from neuroflight_trainer.gyms.step import StepEnv
from neuroflight_trainer.gyms.continuous import ContinuousEnv


class DeltaEnv(ContinuousEnv):
    """An environment in which the action provided by the agent is used as
    a delta in the control output. 

    This environment was motivated by the need to reduce high frequency 
    oscillations in the control output. By requiring the agent to provide a 
    delta, we can constrain how much the control output will be changed and 
    thus attempt to reduce the oscillations. 

    The problem is the constrains ulitimately minimizes the flight
    envelope. A controller will typically output 100% to the actuator at the 
    beginning of maneuver to create a moment. This would not be possible with 
    this type of controller.
    """
    def __init__(self,  
                 max_rate = 1200, 
                 width = 2,
                 max_sim_time = 60,
                 aircraft_config=None,
                 action_bounds = [0, 1],
                 state_fn = None,
                 max_delta = 0.1,
                 state_memory_size = 1,
                 pulse_width = 2,
                 version = 1
                 ): 
        super().__init__(
            aircraft_config = aircraft_config,
            action_bounds = action_bounds,
            #width = width,
            max_sim_time = max_sim_time,
            state_fn = state_fn,
            max_rate = max_rate,
            pulse_width = pulse_width
        )
        self.version = version
        self.state_memory_size = state_memory_size
        self.max_delta = max_delta
        self.state_history = []
        self.last_delta = np.zeros(4)
        
        self.distance = np.zeros(3)
        self.last_distance = np.zeros(3)

        self.at_zero_count = 0

    def step(self, action):
        self.action = action 
        if not self.np_random:
            seed = int(time.time()* 1e6) 
            self.seed(seed)

        delta = np.zeros(4)
        ac = np.clip(action, -1, 1)
        i = 0
        for a in ac:
            if -0.5 < a and a < 0.5: # -0.5 < a < 0.5
                delta[i] = 0
            elif a <= -0.5:
                a += 0.5
                delta[i] = -self.action_to_control_signal(np.abs(a), 0, 0.5, y_low=0, y_high=self.max_delta)

            elif a >= 0.5:
                a -= 0.5
                delta[i] = self.action_to_control_signal(np.abs(a), 0, 0.5, y_low=0, y_high=self.max_delta)
            i+= 1
        self.y = np.clip(self.y + delta, 0, 1) 
        self.command_ys.append(self.y)

        min_throttle = 0.1 # this should have the drone hover
        y = np.minimum(min_throttle + self.y, np.ones(4))
        self.obs = self.step_sim(self.y)

        self.true_error = self.omega_target - self.imu_angular_velocity_rpy
        self.measured_error = self.true_error.copy()
        self.gyro = self.imu_angular_velocity_rpy.copy()

        if self.noise_sigma != 0:
            self.gyro += self.np_random.normal(0, self.noise_sigma, size=3)
            self.measured_error = self.omega_target - self.gyro


        done = self.sim_time >= self.max_sim_time


        reward = self.compute_reward()
        # The agent needs to apply negative output, help them go down
        if self.overshoot(amount_over = np.abs(self.omega_target)*0.3).any():
            a = self.action.copy()
            above_half = a[a >= 0.5]
            if len(above_half) > 0:
                step_time =  (self.sim_time-self.command_start_time) 
                reward -= np.average(above_half) * (self.pulse_width - step_time)

        # give a bit of a reward if not maxing out
        #reward += (20 * np.sum(self.max_delta - delta))
        # if what is added will saturate output
        #over = -1e9 *  np.sum(np.maximum((self.y + delta) - np.ones(4), np.zeros(4)) )
        
        #under= -1e9 *  np.sum(np.abs(np.minimum((self.y + delta), np.zeros(4)) ))

        self.setpoint_generator()
        # call after state is called
        state = self.state_fn(self)

        self.last_distance = self.distance.copy()
        self.last_true_error = self.true_error
        self.iteration += 1

        self.last_y = self.y.copy()
        self.last_delta = delta.copy()
        self.last_gyro = self.gyro.copy()

        self.dbg = {
            "ac": action,
            "delta": delta
        }
        return state, reward, done, self.dbg

    def reward(self, delta):
        step_time =  (self.sim_time-self.command_start_time) 
        min_ac = 0
        min_y = 0
        # The agent needs to apply negative output, help them go down
        if self.overshoot(amount_over = np.abs(self.omega_target)*0.3).any():
            a = self.action.copy()
            above_half = a[a >= 0.5]
            if len(above_half) > 0:
                min_ac -= np.average(above_half) * (self.pulse_width - step_time)

        if (self.omega_target == np.zeros(3)).all():
            # At rpy=0, this can go to zero because
            # we have the min throttle that should keep it 
            # balancedkk
            min_y = -1e6 *  np.max(self.y)# * step_time

        oversaturation_penalty = -1e9 *  np.sum(np.maximum((self.y + delta) - np.ones(4), np.zeros(4)) )# * step_time

        d_y = -1e3 * np.max(np.abs(self.y - self.last_y)) * (step_time > 0.25)
        rewards = [
            min_y,
            d_y,

            min_ac,
            # reduce error
            -np.sum(np.abs(self.true_error)**2),# * step_time,
            # things that should never happen
            # Need this or it wont train
            self.doing_nothing_penalty(),
            oversaturation_penalty,
        ]
        return rewards


    def reset(self):
        self.state_history = []
        self.at_zero_count = 0
        return super().reset()
