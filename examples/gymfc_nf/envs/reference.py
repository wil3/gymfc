import math
import numpy as np
import gym
from gym import spaces
from gym.utils import seeding
from gymfc.envs.fc_env import FlightControlEnv
import time

class ReferenceModel(FlightControlEnv, gym.Env):
    PWM_TOL = 1 #+- 1 microseconds
    GOAL_TARGET = 0
    GOAL_RATE_OSCILLATION = 1

    def __init__(self,  
                 omega_bounds = [-800, 800], 
                 max_command_change = 0,
                 max_sim_time = 30., 
                 command_time_range=[0.5, 2.0],
                 aircraft_config=None,
                 min_acceleration=1000, # What is the acceleration in deg/s^2?
                 action_bounds = [0, 1],
                 num_inputs = 6
                 ): 
        """Create a training environment in which the agent learns to follow a
        pre-calculated step response in the shape of a tanh function.  

        The use of a reference model has potential but this implementation
        results in the agent just tracking a pre-calculated reference. It is
        worth investigating using the damping ratio and frequency response as
        reward terms to characterize the desired performance of the controller.
        """

        self.noise_sigma = 0

        self.action_bounds = action_bounds
        self.np_random = None
        self.GOAL = self.GOAL_TARGET
        self.command_counter = 0
        self.target_reached_goal = 12
        self.target_reached_counter = 0


        self.dbg = {}
        self.iteration = 0
        self.command_iteration = 0

        self.oscillation_reward_frequency = 100

        self.min_acceleration = min_acceleration
        self.velocity_final_deadline = []
        self.max_command_change = max_command_change


        self.max_rpm = 20952

        self.omega_bounds = omega_bounds
        self.max_sim_time = max_sim_time

        # Used to keep track of the outputs so it can be used to 
        # analyze the oscillations
        self.output_history = []
        self.output_history.append(np.zeros(4))
        self.target_reached = np.array([False, False, False])
        self.y = np.zeros(4)

        self.begin_command_time = 0
        self.next_command_step  = 0

        self.step_rates = []
        self.step_u = []

        super().__init__(aircraft_config=aircraft_config, verbose=False)

        self.action_space = spaces.Box(-np.ones(4), np.ones(4), dtype=np.float32)
        self.action = self.action_space.low 

        # TODO make this dynamic based on whats enabled
        # self.num_inputs = 3
        
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(num_inputs,), dtype=np.float32)

        self.command_sent = False

        # Initialize to some random velocity
        self.omega_target = [] #self.sample_target() 

        # COnvert times to steps
        self.command_step_range = [command_time_range[0]/self.stepsize, command_time_range[1]/self.stepsize]

        # Store a 2d array of each target, index by the current ref step
        self.ref = []
        self.tau = 0.2 
        self.last_omega_target = np.zeros(3)



        self.measured_error = np.zeros(3)
        self.true_error = np.zeros(3)
        self.last_measured_error = np.zeros(3)
        self.last_true_error = np.zeros(3)
        self.imu_angular_velocity_rpy = np.zeros(3)
        self.initial_velocity = np.zeros(3)


    def action_to_control_signal(self, action, action_low, action_high):
        """ Action output is going to be the NN output which if 
        from tanh will be [-1:1] and will need to be converted to [0:1]"""

        y_low = 0
        y_high = 1

        ac = np.clip(action, action_low, action_high)
        return ((y_high - y_low) * (ac - action_low) / (action_high - action_low)) + y_low
        
    def gen_ref(self):

        relative_step = np.array(list(range(self.next_command_step - self.iteration))) * self.stepsize

        ref_r = self.last_omega_target[0]*np.exp(-relative_step/self.tau) + self.omega_target[0] * (1 - np.exp(-relative_step/self.tau))
        ref_p = self.last_omega_target[1]*np.exp(-relative_step/self.tau) + self.omega_target[1] * (1 - np.exp(-relative_step/self.tau))
        ref_y = self.last_omega_target[2]*np.exp(-relative_step/self.tau) + self.omega_target[2] * (1 - np.exp(-relative_step/self.tau))
        
        self.ref = np.array(list(zip(ref_r, ref_p, ref_y)))

    def step(self, action):
        self.action = action 
        # XXX seed must be called to initialize the RNG which means
        # this can't be set in the constructor
        # Set it here if the user didn't call reset first.
        #

        if not self.np_random:
            seed = int(time.time()* 1e6) 
            self.seed(seed)

        if len(self.omega_target) == 0:
            self.last_omega_target = self.omega_target
            self.omega_target = self.sample_target()

        if self.next_command_step == 0:
            self.command_iteration = 0
            self.next_command_step = self.np_random.randint(*self.command_step_range)
            #print ("Setting next command at ", self.next_command_step, " Range, ", self.command_step_range)
            self.gen_ref()



            # t = Vf - Vi/a
            self.velocity_final_deadline =  np.abs(self.omega_target - self.imu_angular_velocity_rpy)/self.min_acceleration
            self.begin_command_time = self.sim_time

        #self.y = action
        self.y = self.action_to_control_signal(action, self.action_bounds[0], self.action_bounds[1])

        self.output_history.append(self.y)
        self.obs = self.step_sim(self.y)



        self.true_error = self.omega_target - self.imu_angular_velocity_rpy
        if self.noise_sigma != 0:
            self.measured_error = self.true_error + self.np_random.normal(0, self.noise_sigma, size=3)
        else:
            self.measured_error = self.true_error


        done = self.sim_time >= self.max_sim_time



        self.step_rates.append(self.imu_angular_velocity_rpy)
        self.step_u.append(self.y)

        self.target_reached |= self.is_target_reached()

        reward = self.compute_reward()

        self.check_issue_command()
        



        # call after state is called
        state = self.state()
        self.last_measured_error = self.measured_error
        self.last_true_error = self.true_error

        self.command_iteration += 1
        self.iteration += 1

        return state, reward, done, {}

    def check_issue_command(self):
        if self.iteration >= self.next_command_step-1: # Issue new command

            self.command_iteration = 0
            self.next_command_step = self.iteration + self.np_random.randint(*self.command_step_range)
            #print ("Setting next command at ", self.next_command_step)
            self.last_omega_target = self.omega_target
            self.omega_target = self.sample_target() 
            self.gen_ref()
            self.begin_command_time = self.sim_time
            self.velocity_final_deadline =  np.abs(self.omega_target - self.imu_angular_velocity_rpy)/self.min_acceleration

            self.target_reached = np.array([False, False, False])

            self.initial_velocity = self.imu_angular_velocity_rpy
            self.command_counter += 1


    def is_target_reached(self):
        threshold = np.maximum(
                                            np.abs(self.omega_target) * 0.1,
                                            np.array([5, 5, 5])
        ) 
        return ((np.abs(self.imu_angular_velocity_rpy) >= (np.abs(self.omega_target) - threshold) ) &
            (np.sign(self.imu_angular_velocity_rpy) == np.sign(self.omega_target)))

    def oscillation_u_penalty(self):
        signal = np.array(self.step_u)# * 1000.0
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
            i = np.argmax(prod)
            #print (" Motor f=", f[i], " A=", amp[i])

            #Penalize for any freq greater than cuttoff
            f_cutoff = 1
            index = 0
            for i in range(len(f)):
                if f[i] > f_cutoff:
                    index = i
                    break
            penalty += np.sum(prod[index:])
        return -penalty


    def oscillation_penalty(self):
        """
        Minimize any f > 1
        """
        signal = np.array(self.step_rates)
        r = signal[:,0]
        p = signal[:,1]
        y = signal[:,2]

        N = len(r) 
        T = self.stepsize
        penalty = 0
        for ax in [r,p,y]:
            f = np.linspace(0, 1 / T, N)[:N // 2]
            fft = np.fft.fft(ax)
            amp = np.abs(fft)[:N // 2] * 1 / N

            index = 0
            # Penalize all frequencies greater than 1
            for i in range(len(f)):
                if f[i] > 1:
                    index = i
                    break
            prod = f * amp


            penalty += np.sum(prod[index:])
        return -penalty

    def overshoot(self):
        overshot = ((np.abs(self.imu_angular_velocity_rpy) > (np.abs(self.omega_target)) ) &
            (np.sign(self.imu_angular_velocity_rpy) == np.sign(self.omega_target)))
        return overshot

        #overshot_amount_rpy = (np.abs(self.imu_angular_velocity_rpy) - np.abs(self.omega_target)) * overshot
        #assert (overshot_amount_rpy >= 0).all()
        #return overshot_amount_rpy
        
    def compute_reward(self):
        ref_target = np.round(self.ref[self.command_iteration])
        ref_error = ref_target - self.imu_angular_velocity_rpy

        percent_error = 100 * np.clip(np.average(np.abs(ref_error))/self.omega_bounds[1], 0, 1)
        
        rpm_percent = 100 * np.clip(np.average(self.esc_motor_angular_velocity)/self.max_rpm, 0, 1)

        y_percent = 100 * np.amax(self.y)

        #XXX clip and normalize slope to 2
        output_diff = np.abs(np.diff(self.output_history[-2:], axis=0))
        max_diff = 1/3. 
        delta_output_percent = 100 * np.minimum(np.amax(output_diff)/max_diff, 1) 
        #print ("Out diff=", output_diff, " %=", delta_output_percent)


        #return -(np.power(percent_error, 2))# + (0.05 * y_percent) + (0.1 * delta_output_percent))
        #return -(np.power(percent_error, 2) + (0.05 * y_percent))
        return -np.sum(np.power(ref_error, 2))


    def damping_ratio(self):
        pass

    def half_quadratic_gain(self, fl, fu, fp):

        a = 4 * np.power((fu - fl)/fp, 2)
        b = np.power((fu - fl)/fp, 4)
        c = 0.5 - np.sqrt(np.power(4 + a - b, -1))

        return np.sqrt(c)

    def sample_target(self):
        """ Sample a random angular velocity """
        if not self.np_random:
            seed = int(time.time()* 1e6) 
            self.seed(seed)
        #delta = np.random.randint(0, self.max_command_change, size=3) * np.random.choice([-1, 1], 3)
        # Make sure we dont exceed these thresholds
        # Beta is [0-1] scale to [-1, 1]
        return np.round((self.np_random.beta(2,2, size=3)-0.5)* 2 * self.omega_bounds[1])

    def state(self):
        """ Get the current state 
        
        ob = [angular velocity, accel, orientation, rpm]
        """
        state = self.obs.copy()
        error_delta = (self.measured_error - self.last_measured_error)
        return np.concatenate([self.measured_error, error_delta]) 

    def state2(self):
        """ Get the current state 
        
        ob = [angular velocity, accel, orientation, rpm]
        """
        state = self.obs.copy()
        # 0..2 angular velocity
        state[:3] -= self.omega_target
        # Acceleration
        
        # Orientation [-1, 1]

        # Motor Velocity [0 , max rpm]
        return np.concatenate([state[:3], state[6:]]) 

        #XXX Testing Tensorforce on limited state 
        #return np.clip(state[:3]/self.omega_bounds[1], -1, 1)

        #return state 

    def reset(self):
        self.dbg = {}
        self.measured_error = np.zeros(3)
        self.last_measured_error = np.zeros(3)
        self.true_error = np.zeros(3)
        self.last_true_error = np.zeros(3)
        self.y = np.zeros(4)

        self.omega_target = self.sample_target() 

        self.target_reached = np.array([False, False, False])
        self.output_history = []
        self.output_history.append(np.zeros(4))
        self.step_rates = []
        self.step_u = []
        self.velocity_final_deadline = []
        self.obs = super().reset()
        self.measured_error = self.omega_target - self.imu_angular_velocity_rpy

        self.initial_velocity = np.zeros(3)
        self.begin_command_time = 0
        self.next_command_step = 0

        self.iteration = 0
        self.command_iteration = 0
        return self.state()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

