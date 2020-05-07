# Gymfc_NF

This directory contains the user-module implementations used in Wil Koch's 
[thesis](http://wfk.io/docs/WilliamKochThesisFINAL.pdf) "Flight Controller Synthesis Via Deep Reinforcement Learning".
Specifically it consists of a Python packaged called gymfc_nf which can be used
to synthesize neuro-flight controllers for use in [Neuroflight](https://wfk.io/neuroflight). 


# Installation

1.  Install GymFC_NF, from this directory run,
```
pip3 install .
```
2. (Optional) To train via PPO, first install tensorflow,
```
pip3 install tensorflow
```
Next clone Wil's fork of OpenAI Baselines available
   [here](https://github.com/wil3/openai-baseline) that has modifications to
save Tensorflow checkpoints. In the same virtual environment you use for GymFC, navigate to the directory you cloned the OpenAI Baselines fork and installed it,
```
pip3 install .
```


# Aircraft model / digital twin

The [NF1](https://rotorbuilds.com/build/15163) first person view (FPV) racing
quadcopter model can be found in `gymfc_nf/twins/nf1`. To use this model with gymfc you must
provide the path to `model.sdf`.  

# Controller examples

Two controller examples are provided, a PID controller (`pid_example.py`) and a neuro-controller trained via PPO (`ppo_example.py`).

The PID controller has been tuned using the Ziegler-Nichols method. This tune
has poor performance, this method is known to cause significant overshoot and should only be used as a baseline or to validate functionality of the environment. 

The neuro-controller is synthesized via PPO to generate an optimal controller
based on the provided reward function. During training a checkpoint directory
is created containing snapshots of the neuro-controller. Typically you would want to monitor the checkpoints directory and evaluate new checkpoints on the fly so you can track training performance. 

# Additional Notes
* There are a number of states defined in gymfc_nf/envs/states.py that were
  compiled for experimentation along with some additional environments
(e.g.,delta.py, ramp.py, reference.py). These did not provide the desired
performance however others my find the methods used beneficial for other
applications.   
* There is a policy interface `gymfc_nf/policies` to provide a common interface
  for evaluating controllers of different structures, e.g., PB format, graph
instance, etc. If you intend to synthesize controllers for Neuroflight it is
important to evaluate you checkpoint file to verify the behavior.  
