import math
import numpy as np
from .gazebo_env import GazeboEnv
import logging
logger = logging.getLogger("gymfc")


class AttitudeFlightControlEnv(GazeboEnv):
    def compute_reward(self):
        """ Compute the reward """
        return -np.clip(np.sum(np.abs(self.error))/(self.omega_bounds[1]*3), 0, 1)

    def sample_target(self):
        """ Sample a random angular velocity """
        return  self.np_random.uniform(self.omega_bounds[0], self.omega_bounds[1], size=3)
    
class GyroErrorFeedbackEnv(AttitudeFlightControlEnv):
    def __init__(self, world="attitude-iris.world", 
                 omega_bounds = [-math.pi, math.pi], 
                 max_sim_time = 1.,
                 motor_count = 4, 
                 memory_size=1,): 
        
        self.omega_bounds = omega_bounds
        self.max_sim_time = max_sim_time
        self.memory_size = memory_size
        self.motor_count = motor_count
        self.observation_history = []
        super(GyroErrorFeedbackEnv, self).__init__(motor_count = motor_count, world=world)
        self.omega_target = self.sample_target()

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high) 
        # Step the sim
        self.obs = self.step_sim(action)
        self.error = self.omega_target - self.obs.angular_velocity_rpy
        self.observation_history.append(np.concatenate([self.error]))
        state = self.state()
        done = self.sim_time >= self.max_sim_time
        reward = self.compute_reward()
        info = {"sim_time": self.sim_time, "sp": self.omega_target, "current_rpy": self.omega_actual}

        return state, reward, done, info


    def state(self):
        """ Get the current state """
        # The newest will be at the end of the array
        memory = np.array(self.observation_history[-self.memory_size:])
        return np.pad(memory.ravel(), 
                      ( (3 * self.memory_size) - memory.size, 0), 
                      'constant', constant_values=(0)) 


class GyroErrorESCVelocityFeedbackEnv(GazeboEnv):
    def __init__(self, world="attitude-iris.world", 
                 omega_bounds =[-math.pi, math.pi], 
                 max_sim_time = 1., 
                 motor_count = 4, 
                 memory_size=1,): 
        
        self.omega_bounds = omega_bounds
        self.max_sim_time = max_sim_time
        self.memory_size = memory_size
        self.motor_count = motor_count
        self.observation_history = []
        super(GyroErrorESCVelocityFeedbackEnv, self).__init__(motor_count = motor_count, world=world)
        self.omega_target = self.sample_target()

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high) 
        # Step the sim
        self.obs = self.step_sim(action)
        self.error = self.omega_target - self.obs.angular_velocity_rpy
        self.observation_history.append(np.concatenate([self.error, self.obs.motor_velocity]))
        state = self.state()
        done = self.sim_time >= self.max_sim_time
        reward = self.compute_reward()
        info = {"sim_time": self.sim_time, "sp": self.omega_target, "current_rpy": self.omega_actual}

        return state, reward, done, info

    def compute_reward(self):
        """ Compute the reward """
        return -np.clip(np.sum(np.abs(self.error))/(self.omega_bounds[1]*3), 0, 1)

    def sample_target(self):
        """ Sample a random angular velocity """
        return  self.np_random.uniform(self.omega_bounds[0], self.omega_bounds[1], size=3)

    def state(self):
        """ Get the current state """
        # The newest will be at the end of the array
        memory = np.array(self.observation_history[-self.memory_size:])
        return np.pad(memory.ravel(), 
                      (( (3+self.motor_count) * self.memory_size) - memory.size, 0), 
                      'constant', constant_values=(0)) 


class GyroErrorESCVelocityFeedbackContinuousEnv(GyroErrorESCVelocityFeedbackEnv):
    def __init__(self, command_time_off=[], command_time_on=[], **kwargs):
        self.command_time_off = command_time_off
        self.command_time_on = command_time_on
        self.command_off_time = None
        super(GyroErrorESCVelocityFeedbackContinuousEnv, self).__init__(**kwargs)

    def step(self, action):
        """ Sample a random angular velocity """
        ret = super(GyroErrorESCVelocityFeedbackContinuousEnv, self).step(action) 

        # Update the target angular velocity 
        if not self.command_off_time:
            self.command_off_time = self.np_random.uniform(*self.command_time_on)
        elif self.sim_time >= self.command_off_time: # Issue new command
            # Commands are executed as pulses, always returning to center
            if (self.omega_target == np.zeros(3)).all():
                self.omega_target = self.sample_target() 
                self.command_off_time = self.sim_time  + self.np_random.uniform(*self.command_time_on)
            else:
                self.omega_target = np.zeros(3)
                self.command_off_time = self.sim_time  + self.np_random.uniform(*self.command_time_off) 

        return ret 
        

class GyroErrorESCVelocityFeedbackEnv_1(GazeboEnv):
    def __init__(self, world="attitude-iris_1.world", 
                 omega_bounds =[-math.pi, math.pi], 
                 max_sim_time = 1., 
                 motor_count = 4, 
                 memory_size=1,): 
        
        self.omega_bounds = omega_bounds
        self.max_sim_time = max_sim_time
        self.memory_size = memory_size
        self.motor_count = motor_count
        self.observation_history = []
        super(GyroErrorESCVelocityFeedbackEnv_1, self).__init__(motor_count = motor_count, world=world)
        self.omega_target = self.sample_target()

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        # Step the sim
        self.obs = self.step_sim(action)
        self.observation_history.append(np.concatenate([self.obs.velocity_xyz,self.obs.angular_velocity_rpy]))
        state = self.state()
        self.error = self.omega_target - state
        #state=self.state_new
        #print("obs :{}, tar :{}".format(self.omega_target))
        done_1 = self.sim_time >= self.max_sim_time
        done_2 = ((state-self.omega_target)**2).mean(axis=0)<=0.01
        done = done_1 or done_2
        reward = self.compute_reward()
        info = {"sim_time": self.sim_time, "sp": self.omega_target, "current_rpy": self.omega_actual, "velocity": self.obs.velocity_xyz}

        return state, reward, done,info

    def compute_reward(self):
        """ Compute the reward """
        #return -np.clip(np.sum(np.abs(self.error))/(self.omega_bounds[1]*3), 0, 1)
        diag=np.array([1,0.75,0.75,0,0.25,1])
        diag=np.diag(diag)
        et=np.array(self.error)
        r_t=et.dot(diag)
        r=r_t.dot(et)
        return -r

    def sample_target(self):
        """ Sample a random angular velocity """
        #return  self.np_random.uniform(self.omega_bounds[0], self.omega_bounds[1], size=3)
        return np.array([0.1,0.2,-0.1,0,0,0])
    def state(self):
        damn=np.array([0,0,0,0,0,0])
        if len(self.observation_history)!=0:
            damn = self.observation_history.pop(-1)
            damn = np.array(damn)
        return damn
'''
    def state(self):
        """ Get the current state """
        # The newest will be at the end of the array
        memory = np.array(self.observation_history[-self.memory_size:])
        return np.pad(memory.ravel(), 
                      (( (6+self.motor_count) * self.memory_size) - memory.size, 0), 
                      'constant', constant_values=(0))
'''



