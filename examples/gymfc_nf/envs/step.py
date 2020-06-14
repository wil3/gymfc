import math
import numpy as np
import gym
from gym import spaces
from gym.utils import seeding
from gymfc.envs.fc_env import FlightControlEnv
import time
from .rewards import RewardEnv

class StepEnv(RewardEnv): 
    def __init__(self, pulse_width = 1, max_rate = 100, state_fn = None,
                 max_sim_time = 1 ):
        """Create a reinforcement learning environment that generates step input
        setpoints. Technically this is a multi-axis singlet input, the
        terminology in this package needs to be updated to reflect flight test
        maneuvers.

        This environment was created to teach an agent how to respond to
        worst-case inputs, that is, step inputs in which there is a request for
        immediate change in the target angular velocity.

        Start at zero deg/s to establish an initial condition and teach the
        agent to idle. Sample random input and hold for pulse_width, then
        return to zero deg/s to allow system to settle.

        Args:
            pulse_width: Number of seconds the step is held at the target 
                setpoint.
            max_rate: Max angular rate to sample from, or in the case of a 
                normal distribution, the mean. 
            state_fn: See BaseEnv
            max_sim_time: See BaseEnv
        """

        super().__init__(max_sim_time = max_sim_time, state_fn = state_fn)

        self.pulse_width = pulse_width
        self.max_rate = max_rate

        self.rising = True
        self.outputs = []
        self.angular_rate_sp = np.zeros(3)
        self.next_pulse_time = 0.512


    def update_setpoint(self):
        if self.sim_time > self.next_pulse_time:
            if (self.angular_rate_sp == np.zeros(3)).all():
                self.angular_rate_sp = self.generated_input
                self.next_pulse_time += self.pulse_width
            else:
                self.angular_rate_sp = np.zeros(3)
                self.next_pulse_time += self.pulse_width
            self.rising = False

    def reset(self):
        self.rising = True
        self.outputs = []
        self.angular_rate_sp = np.zeros(3)
        self.next_pulse_time = 0.512
        # Define the singlet input in the beginning so it can be overriden
        # externally if needed for testing.
        self.generated_input = self.sample_target()
        return super().reset()

    def sample_target(self):
        """Sample a random angular velocity setpoint """
        return self.np_random.normal(0, self.max_rate, size=3)


