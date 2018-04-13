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

1. Download and install [Gazebo 8](http://gazebosim.org/download).
2. From root directory of this project, `sudo pip3 install -e .`


# Environments

## QuadcopterFCEpisodic-v0

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

## QuadcopterFCContinuous-v0

This environment is essentially the same as the episodic variant however it runs
for 60 seconds and continually changes the target angular velocities randomly
between [0.1, 1] seconds.
