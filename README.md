# GymFC

GymFC is a simulation environment for developing flight control systems. 
environment is meant to serve as a tool for researchers to benchmark their
controllers to progress the state-of-the art of flight control.
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

# Features

* Support for IMU and ESC sensors
* Aircraft agnostic - support for any type of aircraft just configure number of
  actuators and sensors.
* Digital twin independence - digital twin is developed external to GymFC
  allowing separate versioning.
* Google protobuf aircraft digital twin API for publishing control
  signals and subscribing to sensor data. 
* Flexible agent interface allowing controller development for any type of flight control systems.
* Compatible with OpenAI environments.
* Support for Gazebo 8 and 9. Gazebo plugins are built dynamically depending on
  your installed version. 
* 
* Configurable through JSON


# News

* March 2019 - GymFC v0.2.0 is released. 
The new GymFC version is a major rewrite with substantial architectural
change emphasizing core princiles such as ocntroller development for a specific aircraft.
GymFC is now essentially a generic flight control environment middleware between the aircraft digital twin and the agent interface. Users now are responsible for implementing the agent interface and the digital twin; which are both unique to the aircraft.
* December 2018 - Our GymFC manuscript is accepted to the journal ACM Transactions on Cyber-Physical Systems.
* November 2018 - Flight controller synthesized with GymFC achieves stable
  flight in [Neuroflight](https://github.com/wil3/neuroflight).
* September 2018 - GymFC v0.1.0 is released.
* April 2018 - Pre-print of our paper is published to [arXiv](https://arxiv.org/abs/1804.04154). 


# Installation 

Note, Ubuntu is the only OS currently supported. Please submit a PR for the
README.md if you are
able to get it working on another platform.   
1. Download and install [Gazebo 9](http://gazebosim.org/download). Tested on
   Ubuntu 16.04 LTS and 18.04 LTS.  Make sure you also install the dev package, `sudo apt-get install libgazebo9-dev` which will be used to build the Gazebo plugins.
2. (Optional) It is suggested to set up a [virtual environment](https://docs.python.org/3/library/venv.html). From the project root,
   `python3 -m venv env`. This will create an environment named `env` which
will be ignored by git. To enable the virtual environment, `source
env/bin/activate` and to deactivate, `deactivate`.  
2. From root directory of this project, `pip3 install .`
3. Rename `gymfc.json.template` to `gymfc.json`

# Verifying Installation 
To verify you have installed the environment correctly it is recommended to run
the PID controller
https://github.com/wil3/gymfc-envs 

controlling an Iris quadcopter model.

his example
uses the configuration file `examples/config/iris.config`. Before running the
example verify the configuration, specifically that the Gazebo `SetupFile` is pointing to the correct location.
The example requires additional Python modules, from `examples` run,
```
pip3 install -r requirements.txt
```
To run the example change directories to `examples/controllers` and execute,
```
python3 run_iris_pid.py
```
If your environment is installed properly you should observe a plot that
closely resembles this step response,
![PID Step
Response](https://raw.githubusercontent.com/wil3/gymfc/master/images/pid-step-AttFC_GyroErr-MotorVel_M4_Ep-v0.png)


# Development 

GymFC's primary goal is to train controllers capable of flight in the real-world. 
 In order to construct optimal flight controllers, the aircraft used in
simulation should closely match the real-world aircraft. Therefore the GymFC environment is decoupled from the simulated aircraft.
As previously mentioned, GymFC comes with an example to verify the environment.
The Iris model can be useful for testing out new controllers. However when
transferring the controller to run on a different aircraft, a new model will be
required. Once the model is developed set the model directory to `AircraftModel` in your configuration file.

It is recommended to run GymFC in headless mode (i.e. using `gzserver`) however
during development and testing it may be desired to visually see the aircraft.  You can do this by using the `render` OpenAI gym API call which will also start `gzclient` along side `gzserver`. For example when creating the environment use,
```
env = gym.make(env_id)
env.render()
```
[![GymFC Visualization Demo](https://raw.githubusercontent.com/wil3/gymfc/master/images/gymfc-vis.png)](https://youtu.be/sX9NwmDg6SA)

If you plan to work with the GymFC source code you will want to install it in
development mode, `pip3 install -e .` from the root directory. You will also
need to build the plugin manually by running the script
`gymfc/envs/assets/gazebo/plugins/build_plugin.sh`. 


## Digital twin interface

Center of thrust
## Agent Interface

