import math
import numpy as np
import gym
from gym import spaces
from gym.utils import seeding
from gymfc.envs.fc_env import FlightControlEnv
import time

class BaseEnv(FlightControlEnv, gym.Env):
    def __init__(self, max_sim_time = 30, state_fn = None): 
        """ Create a base gymfc environment.

        Environments should derive this class to implement their own setpoint
        generators. 

        After initializing this environment the agent should call seed() 
        and then reset() so the seed will be used for sampling the desired 
        state.

        Args:
            max_sim_time: The maximum time in seconds the simulator will run.
            state_fn: Function returning a numpy array defining the environment 
                state. This is to provide additional flexability testing various 
                states. 
        """

        self.max_sim_time = max_sim_time
        self.state_fn = state_fn

        # Instance of the RNG used for all derived classes
        self.np_random = None

        # Init all of the variables for the simulation
        self._init()

        # Define the Gym action and observation spaces
        self.action_space = spaces.Box(-np.ones(4), np.ones(4), dtype=np.float32)
        self.action = self.action_space.low 
        num_inputs = len(self.state_fn(self))
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(num_inputs,), 
                                            dtype=np.float32)

        # IMU noise is aircraft specific and thus can not be included as
        # initial values for the registered environment. After the environment
        # has been created the user can update this function.
        self.sample_noise = lambda _: 0

        # A callback made at the end of each step. It takes a single
        # parameter containing the class reference
        self.step_callback = None

    def set_aircraft_model(self, model):
        """Set the aircraft's model.sdf
        
        Args:
            model: Absolute path to the digital twins model.sdf file.
        """

        # XXX Calling the parent class is delayed because of how OpenAI gym 
        # initializes environments through the registered environment IDs. 
        # Since we don't want to keep hardcoded paths in our environment 
        # registration, we finish the configuration after the environment is 
        # created. When gymfc is created it will dynamically load the 
        # digital twin at the same time.
        super().__init__(aircraft_config=model)

    def step(self, action):
        """Step the simulator and apply the provided action. 

        Args:
            action: numpy array where each value in the array is the action 
                indexed by the acutuator index defined in the models SDF.
        """
        self.action = action.copy()

        # Translate the agents output to the aircraft control signals. In this
        # case our control signal is represented as a percentage. This 
        # function also needs to exist in the flight control firmware. 
        self.y = self.action_to_control_signal(self.action, 0, 1)

        # Interface with gymfc
        self.obs = self.step_sim(self.y)

        self.angular_rate = (self.imu_angular_velocity_rpy.copy() + 
            self.sample_noise(self))
        self.true_error = self.angular_rate_sp - self.imu_angular_velocity_rpy
        self.measured_error = self.angular_rate_sp - self.angular_rate

        done = self.sim_time >= self.max_sim_time 

        reward = self.compute_reward()

        # Generate the next setpoint
        self.update_setpoint()

        # And the current state for the agent which will be usd as input to the 
        # neural network.
        state = self.state_fn(self)

        self.last_measured_error = self.measured_error.copy() 
        self.last_y = self.y.copy()
        self.step_counter += 1
        if self.step_callback:
            self.step_callback(self, state, reward, done)
        return state, reward, done, {}

    def action_to_control_signal(self, action, action_low, action_high, 
                                 y_low=0, y_high=1):
        """Action output is going to be the NN output which is from tanh will 
        be [-1:1] and will need to be converted to [0:1] which relates to the 
        percent power for each motor
        """

        ac = np.clip(action, action_low, action_high)
        return ((y_high - y_low) * (ac - action_low) / 
                (action_high - action_low)) + y_low

    def _init(self):
        # Used to keep track of the outputs so it can be used to analyze the 
        # oscillations
        self.y = np.zeros(4)
        # Used to calculate the delta in the control output
        self.last_y = np.zeros(4)

        # Measured with noise for the roll, pitch and yaw axis respectively
        self.angular_rate = np.zeros(3)
        # Initialize to some random velocity
        self.angular_rate_sp = np.zeros(3)

        # Used for agent input and has the IMU noise applied as a method for
        # domain randomization.
        self.measured_error = np.zeros(3)
        # Used for calculating delta errors
        self.last_measured_error = np.zeros(3)
        # True error is used for computing rewards
        self.true_error = np.zeros(3)
        self.imu_angular_velocity_rpy = np.zeros(3)
        #self.imu_orientation_quat = np.array([0, 0, 0, 1])

        # Keep track of the number of steps so we can determine how many steps
        # occur in an episode.
        self.step_counter = 0

    def reset(self):
        self._init()
        self.obs = super().reset()

        return self.state_fn(self)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def compute_reward(self):
        raise exception.NotImplementedError

    def update_setpoint(self):
        """Update the angular velocity setpoint, angular_rate_sp."""
        raise exception.NotImplementedError
