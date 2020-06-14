
import numpy as np
from .base import BaseEnv
import math

class RewardEnv(BaseEnv):
    """An environment for reinforcement learning.
    
    At each time step this class calculates and provides the agent with a 
    numberical reward indicating how well the agent did at performing the task.
    The reward must encapsulte the performance of the agent's step response. 

    All reinforcement learning environments should derive this class and
    implement setpoint_generator to define the angular velocities the agent
    should track.

    One of the main challenges synthezing flight controllers via 
    reinforcement learning is developing the reward.  
    """
    def __init__(self, max_sim_time = 30, state_fn = None): 
        """ Create a reinforcment learning environment that provides the agent
        with a reward.

        Args:
           max_sim_time: See BaseEnv 
           state_fn: See BaseEnv
        """

        super().__init__(max_sim_time = max_sim_time, state_fn = state_fn)
        self._reset()


    def compute_reward(self):

        shaping = \
            -np.sum(self.true_error**2)
        
        e_penalty = 0
        if self.prev_shaping is not None:
            e_penalty = shaping - self.prev_shaping
        self.prev_shaping = shaping

        # Reward the agent for minimizing their control ouputs. 
        # In order to get the reward, the agent must be in the error band, 
        # otherwise the agent will just output zero.
        min_y_reward = 0
        # Error band is minimum 5 deg/s, maximum 10% of angular rate sp
        threshold = np.maximum(np.abs(self.angular_rate_sp) * 0.1, np.array([5]*3)) 
        inband = (np.abs(self.true_error) <= threshold).all()
        percent_idle = 0.12 #should be about 12%, different for every aircraft
        max_min_y_reward = 1000
        if np.average(self.y) < percent_idle: # You get a max of 900
            min_y_reward = max_min_y_reward * (1 - percent_idle) * inband
        else:
            min_y_reward = max_min_y_reward * (1 - np.average(self.y)) * inband 

        rewards = [
            # penalty for oscillations
            -1000 * np.max(np.abs(self.y - self.last_y)),
            # Reward for minimizing control output. This is needed  because the
            # training environment fixes the aircraft about its center of thrust
            # thus we need to teach the agent to use as low control output values
            # as possible. 
            #
            # A balance needs to be 
            min_y_reward,
            # penalty for angular velocity tracking
            e_penalty,
            # penalty for oversaturating the control output, without this 
            # the agent tends to generate binary outputs, i.e., [-1, 1].
            -1e9 * np.sum(self.oversaturation_high()),
            # penalty if the agent does nothing, i.e., refusing to 'play'
            self.doing_nothing_penalty(),
        ]
        self.ind_rewards = rewards

        return np.sum(rewards)

    def doing_nothing_penalty(self, penalty=1e9):
        total_penalty = 0 

        # These always apply
        if np.sum(self.y == 0) > 2 and not (self.angular_rate_sp == np.zeros(3)).all():
            total_penalty -= penalty 
        # All high, should only happen on a punch out, not for attitude control
        if (self.y == 1).all():
            total_penalty -= penalty
        return total_penalty

    def oversaturation(self):
        """Return amount over 1 output by the NN/agent"""
        return np.maximum(np.abs(self.action) - np.ones(4), np.zeros(4))

    def oversaturation_high(self):
        """Return amount over 1 output by the NN/agent"""
        ac = np.maximum(self.action, np.zeros(4))
        return np.maximum(ac - np.ones(4), np.zeros(4))

    def overshoot(self, amount_over=np.zeros(3)):
        """Return boolean array size 3 for roll pitch yaw indicating if overshoot """
        overshot = ((np.abs(self.imu_angular_velocity_rpy) + amount_over > (np.abs(self.angular_rate_sp)) ) &
            (np.sign(self.imu_angular_velocity_rpy) == np.sign(self.angular_rate_sp)))
        return overshot

    def _reset(self):
        self.prev_shaping = None

    def reset(self):
        self._reset()
        return super().reset()

    def oscillation_u_penalty(self):
        # XXX Not currently used
        signal = np.array(self.command_ys)# * 1000.0
        m1 = signal[:,0]
        m2 = signal[:,1]
        m3 = signal[:,2]
        m4 = signal[:,3]

        N = len(m1) 
        T = self.stepsize
        penalty = 0
        for m in [m1, m2, m3, m4]:
            f = np.linspace(0, 1 / T, N)[:N // 2]
            fft = np.fft.fft(m)
            amp = np.abs(fft)[:N // 2] * 1 / N
            prod = f * amp
            """
            i = np.argmax(prod)
            #print (" Motor f=", f[i], " A=", amp[i])

            #Penalize for any freq greater than cuttoff
            f_cutoff = 1
            index = 0
            for i in range(len(f)):
                if f[i] > f_cutoff:
                    index = i
                    break
            """
            penalty += np.sum(prod)
        return -penalty


