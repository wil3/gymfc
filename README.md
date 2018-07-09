# GymFC

GymFC is an [OpenAI Gym](https://github.com/openai/gym) environment specifically
designed for
developing intelligent flight control systems using reinforcement learning. This
environment is meant to serve as a tool for researchers to benchmark their
controllers to progress the state-of-the art of intelligent flight control.
Our tech report is available at [https://arxiv.org/abs/1804.04154](https://arxiv.org/abs/1804.04154)  providing details of the
environment and  benchmarking of PPO, TRPO and DDPG using [OpenAI Baselines](https://github.com/openai/baselines). We compare the performance results to a PID controller and find PPO to out perform PID in regards to rise time and overall error. Please use the following BibTex entry to cite our
work,
```
@misc{1804.04154,
Author = {William Koch and Renato Mancuso and Richard West and Azer Bestavros},
Title = {Reinforcement Learning for UAV Attitude Control},
Year = {2018},
Eprint = {arXiv:1804.04154},
}
```

# Installation 
Note, Ubuntu is the only OS currently supported primarily because controllers must be
trained in headless mode on a server. I welcome any PRs and feedback for getting
it installed on other OSs.   
1. Download and install [Gazebo 8](http://gazebosim.org/download) (PRs welcome
   for Gazebo 9).
2. From root directory of this project, `sudo pip3 install -e .`
3. Confirm environment is operating successfully by running an evaluation for
   the
   included PID controller tuned for the Iris quadcopter used in the environment
and referenced paper,
```
python3 -m gymfc.controllers.iris_pid_eval --env-id=AttFC_GyroErr-MotorVel_M4_Ep-v0
```
If your environment is installed successfully you should observe a plot that
closely resembles this step response,
![PID Step
Response](https://raw.githubusercontent.com/wil3/gymfc/master/images/pid-step-AttFC_GyroErr-MotorVel_M4_Ep-v0.png)


# Development 

It is recommended to run GymFC in headless mode (i.e. using `gzserver`) however
during development and testing it may be desired to visually see the aircraft.  You can do this by using the `render` OpenAI gym API call which will also start `gzclient` along side `gzserver`. For example when creating the environment use,
```
env = gym.make(env_id)
env.render()
```
[![GymFC Visualization Demo](https://raw.githubusercontent.com/wil3/gymfc/master/images/gymfc-vis.png)](https://youtu.be/sX9NwmDg6SA)

# Environments

Different environments are available depending on the capabilities of the flight
control system. For example new ESCs contain sensors to provide telemetry
including the velocity of the rotor which can be used as additional state in the
environment. Environment naming format is [prefix]\_[inputs]\_M[actuator
count]\_[task type] where prefix=AttFC, Ep is episodic tasks, and Con is
continuous tasks.

## AttFC_GyroErr-MotorVel_M4_Ep-v0

This environment is an episodic task to learn attitude control of a quadcopter. At the beginning of each episode the
quadcopter is at rest. A random angular velocity is sampled and the agent must achieve this target  within
1 second. 

**Observation Space** Box(7,) where 3 observations correspond to the angular velocity error for each axis in radians/second (i.e Ω\* − Ω) in range [-inf, inf] and 4 observations correspond
to the angular velocity of each rotor in range [-inf, inf].
 
**Action Space** Box(4,) corresponding to each PWM value to be sent to the ESC in
the range [-1, 1].

**Reward** The error normalized between [-1, 0] representing how close the angular velocity is
to the target calculated by -clip(sum(|Ω\* − Ω |)/3Ω\_max)  where the clip
function bounds the result to [-1, 0] and  Ω\_max is the initially error from
when the target angular velocity is set.

Note: In the referenced paper different memory sizes were tested, however for PPO it was
found additional memory did not help. At the moment for research, debugging and testing purposes environments with different memory sizes are included and can be referenced by AttFC_GyroErr1-MotorVel_M4_Ep-v0 - AttFC_GyroErr10-MotorVel_M4_Ep-v0.

## AttFC_GyroErr-MotorVel_M4_Con-v0

This environment is essentially the same as the episodic variant however it runs
for 60 seconds and continually changes the target angular velocities randomly
between [0.1, 1] seconds.

## AttFC_GyroErr1_M4_Ep-v0 - AttFC_GyroErr10_M4_Ep-v0

This environment supports ESCs without telemetry and only relies on the gyro
readings as environment observations. Preliminary testing has shown memory > 1
increases accuracy. 
