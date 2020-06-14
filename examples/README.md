# Gymfc_NF

This directory contains the implementations used in Wil Koch's 
[thesis](http://wfk.io/docs/WilliamKochThesisFINAL.pdf) "Flight Controller Synthesis Via Deep Reinforcement Learning". Specifically it consists of a Python packaged called gymfc_nf which can be used
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
3. Build the motor models. 
```
pip3 install .
mkdir gymfc_nf/twins/nf1/plugins
cd gymfc_nf/twins/nf1/plugins
git clone https://github.com/wil3/gymfc-aircraft-plugins.git
mkdir build
cd build
cmake ../gymfc-aircraft-plugins
make
```


# Aircraft model / digital twin

The [NF1](https://rotorbuilds.com/build/15163) first person view (FPV) racing
quadcopter model can be found in `gymfc_nf/twins/nf1`. To use this model with gymfc you must
provide the path to `model.sdf`.  

# Controller examples
Two controller examples are provided, a PID controller (`pid_example.py`) and a neuro-controller trained via PPO.

## PID Controller

The PID controller has been tuned using the Ziegler-Nichols method. This tune
has poor performance, this method is known to cause significant overshoot and should only be used as a baseline or to validate functionality of the environment. An interesting use of GymFC would be to use optimization algorithms to compute the PID gains for example genetic algorithms. 

To run the example execute,

```
python3 pid_example.py
```
This comman will display a graph showing the step response of the controller.

## PPO Neuro Controller
The neuro-controller is synthesized via PPO to generate an optimal controller
based on the provided reward function. Neuro-controllers are significantly more
complex than traditional linear controllers and thus running them is more
involved.

To train a neuro-controller first execute the trainer,
```
python3 ppo_baselines_train.py
```
This will train a neuro-controller and by default save Tensorflow checkpoints
of the neural network every 100,000 timesteps to
`../../models/<model_name>/checkpoints`.

While we are training we would like to monitor its progress and evaluate each
checkpoint. In a separate window (suggest using tmux or equivalent), execute the monitor
and evaluator,
```
python3 tf_checkpoint_evaluate.py ../../models/<model_name>/checkpoints --num-trials=3
```
This script will monitor the checkpoints directory and when a new checkpoint is
saved it will evaluate the neural network against num-trials number of random
setpoints and save the results to `../../models/<model_name>/evaluations`


We can then plot some metrics of each checkpoint using,
```
python plot_flight_metrics.py ../../models/<model_name>/evaluations
```
and look at specific evaluation trials using,
```
python3 plot_flight.py ../../models/<model_name>/evaluations/<checkpoint name>/trial-X.csv
```

After about 2,000,000 time steps you should start to see the model converge.
Using `plot_flight_metrics.py` and other analysis, select the best checkpoint
to use in Neuroflight. 

## FAQ

1. Why is my model is not converging?

RL is notorious for being difficult to reproduce. Train multiple agents using different seeds and take the best one. Improvements to the reward function can help increase stability and reproducibility. 

2. Why does my model have such high oscillations?

When selecting an agent **low MAE is not everything!** By training to minimize
the error the agent is trying to constantly correct it self like an over tuned
PID controller. The most challenging aspect of this research has been minimizing output oscillations. This has been discussion repeatedly in the reference literature if you like to learn more. Look for agents with minimal 
changes to their output and try it in the real world to verify oscillations are
not visible. There is huge room for improvement here. 

# Research challenges 

For those looking to advance the state-of-the-art in neuro-flight control there are a number of
areas to pursue. Many of these areas are discussed in the referenced thesis. In
terms of motivation for high performance applications such as drone racing, the
greatest challenge this research currently faces is in reducing motor oscillations that occur in the real
world. Some areas to investigate include, 
* Introducing oscillation metrics to the step and motor response such as the damping ratio. 
* Reducing the reality gap between the simulator training world and the real world. 
* Adding additional domain randomization strategies.  


# Additional Notes
* There are a number of states defined in gymfc_nf/envs/states.py that were
  compiled for experimentation along with some additional environments
(e.g., delta.py, ramp.py, reference.py). These did not provide the desired
performance however others my find the methods used beneficial for other
applications. These are provided, as is, for inspiration. These have been left
out of the package refactoring and currently contain errors preventing them
from functioning properly.  Please submit a PR if you are able to finish the
integrating into gymfc_nf. 
* There is a policy interface `gymfc_nf/policies` to provide a common interface
  for evaluating controllers of different structures, e.g., pb format, graph
instance, etc. 
If you intend to synthesize controllers for Neuroflight it is critical to
evaluate the performance during each transformation process to verify the
behavior is expected. The
Neuroflight toolchain will take a checkpoint and convert it to a pb format so
it can then be compiled to a c++ object. It is recommended to test the
performance of this generated pb file, details can be found in the Neuroflight
[make file](https://github.com/wil3/neuroflight/blob/v3.3.x-neuroflight/make/neuroflight.mk). 

