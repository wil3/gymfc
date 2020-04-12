![GymFC](https://github.com/wil3/gymfc/blob/master/images/gymfc-logo.png)

GymFC is flight control tuning framework with a focus in attitude control. 
Intially GymFC was first introduced in the [manuscript](http://wfk.io/docs/gymfc.pdf) "Reinforcement learning for UAV attitude control" in which the simulator was used to
synthesize neuro-flight attitude controllers that exceeded the performance of a traditional PID controller. 
Since the intial release of the project is has matured and become a modular
framework
for tuning flight controllers system, not only for synthesizing neuro-flight
controllers but also tuning traditional controllers as well. 
Currently, GymFC is the primary method for developing controllers to be used in the worlds
first neural network supported
flight control firmware [Neuroflight](https://wfk.io/neuroflight). 
Please use the following BibTex entry to cite our work,
```
@article{koch2019reinforcement,
  title={Reinforcement learning for UAV attitude control},
  author={Koch, William and Mancuso, Renato and West, Richard and Bestavros, Azer},
  journal={ACM Transactions on Cyber-Physical Systems},
  volume={3},
  number={2},
  pages={22},
  year={2019},
  publisher={ACM}
}
```

![Architecture](https://github.com/wil3/gymfc/blob/master/images/gymfc2-arch.png)

> :warning: **Warning:** Documentation is lagging behind in regards to the new
> framework and installation instructions. Migration to GymFC2 is going to take
> some effort however it provides a sustainable framework moving forward.
> Documentation and additional examples will be added with time. Best bet at
> the moment is to read through the code, python side is in good shape, c++
> needs love.  

## Table of contents

* [Features](https://github.com/wil3/gymfc#features)
* [News](https://github.com/wil3/gymfc#news)
* [Installation](https://github.com/wil3/gymfc#installation)
* [Getting Started](https://github.com/wil3/gymfc#getting-started)
* [User Modules](https://github.com/wil3/gymfc#available-user-provided-modules)
* [Custom Modules](https://github.com/wil3/gymfc#custom-user-modules)
* [Development Team](https://github.com/wil3/gymfc#)
* [Contributions](https://github.com/wil3/gymfc#contributions)




# Features

* Support for IMU, ESC and battery sensors
* Aircraft agnostic - support for any type of aircraft just configure number of
  actuators and sensors.
* Digital twin independence - digital twin is developed external to GymFC
  allowing separate versioning.
* Google protobuf aircraft digital twin API for publishing control
  signals and subscribing to sensor data. 
* Flexible agent interface allowing controller development for any type of flight control systems.
* Support for Gazebo 8, 9, and 11. Gazebo plugins are built dynamically depending on
  your installed version. 


# News

* July 2019 - GymFC v0.2.0 is released. 
* December 2018 - Our GymFC manuscript is accepted to the journal ACM Transactions on Cyber-Physical Systems.
* November 2018 - Flight controller synthesized with GymFC achieves stable
  flight in [Neuroflight](https://github.com/wil3/neuroflight).
* September 2018 - GymFC v0.1.0 is released.
* April 2018 - Pre-print of our paper is published to [arXiv](https://arxiv.org/abs/1804.04154). 


# Installation 

Note, Ubuntu 16.04 LTS and 18.04 LTS are the only OS currently supported. Please submit a PR for the
README.md if you are
able to get it working on another platform. To ensure accurate and stable
simulations it is recommended to use DART with Gazebo. This requires Gazebo to
be installed from source. For more information please see this
[video](https://www.youtube.com/watch?v=d3NyFU0bVT0).  We have found these
versions to work well together,
1. Compile and install DART v6.7 from source
   [here](https://github.com/dartsim/dart/tree/v6.7.0).
1. Compile and install [Gazebo 10](http://gazebosim.org/tutorials?tut=install_from_source&cat=install). 
2. (Optional) It is suggested to set up a [virtual environment](https://docs.python.org/3/library/venv.html). From the project root,
   `python3 -m venv env`. This will create an environment named `env` which
will be ignored by git. To enable the virtual environment, `source
env/bin/activate` and to deactivate, `deactivate`.  
2. From root directory of this project, `pip3 install .` If you plan to work with the GymFC source code you will want to install it in
development mode, `pip3 install -e .` You will also
need to build the plugin manually by running the script
`gymfc/envs/assets/gazebo/plugins/build_plugin.sh`.
3. Confirm `SetupFile` in `gymfc.ini` is pointing to the correct location.

## Installation using Docker
This repository includes an experimental docker build that demos the usage of GymFC. 
It has been tested on MacOS 10.14.3 and Ubuntu 18.04, however the Gazebo client
has not been verified to work for Ubuntu. This docker image can help ensure you
are running a supported environment for GymFC.

### Install dependencies 

For Mac, install [Docker for Mac](https://docs.docker.com/docker-for-mac/install/) and [XQuartz](https://www.xquartz.org/) on your system.
For Ubuntu, install [Docker for Ubuntu](https://docs.docker.com/engine/install/ubuntu/).

### Build and test 
Build the docker image

```
docker build  -f Dockerfile-demo . -t gymfc:demo
```
This will take a while as it compiles mesa drivers, gazebo and dart. It is recommended to give Docker a large part of the host's resources.
All incoming connections will forward to xquartz:

```
xhost +
```

Example usage, run the image and test test_step_sim.py using the [Solo digital twin](https://github.com/wil3/gymfc-digitaltwin-solo.git),
```
docker run -ti -e DISPLAY=<hostip>:0 \
-v <path-to-gymfc-digitaltwin-solo>/models/solo/model.sdf:/gymfc/demo/models/solo/model.sdf \
gymfc:demo \
"python3 /gymfc/tests/test_step_sim.py --gymfc-config /gymfc/gymfc.ini --verbose /gymfc/demo/models/solo/model.sdf  1 1 1 1"
```

Replace _<hostip>_ by the external ip of your system to allow gymfc to connect to your XQuartz server and _<path-to-gymfc-digitaltwin-solo>_ to where you cloned the Solo repo.
Take special note that the test_step_sim.py parameters are using the containers
path, not the host's path.


# Getting Started 

The simpilist environment can be created with,

```python
from gymfc.envs.fc_env import FlightControlEnv
class MyEnv(FlightControlEnv):
    def __init__(self, aircraft_config, config=None, verbose=False):
        super().__init__(aircraft_config, config_filepath=config, verbose=verbose)
```

By inheritting FlightControlEnv you now have access to the `step_sim` and
`reset` functions. If you want to create an OpenAI gym you also need to inherit
this class e.g.,

```python

from gymfc.envs.fc_env import FlightControlEnv
import gym
class MyOpenAIEnv(FlightControlEnv, gym.Env):  
```
 
For simplicity the GymFC environment takes as input a single `aircraft_config` which is the file location of your aircraft model  `model.sdf`. The SDF declares all the visualizations, geometries and plugins for the aircraft.

### Directory Layout
GymFC expects your model to have the following Gazebo style directory structure: 
```
model_name/
  model.config
  model.sdf
  plugins/
    build/
```
where the `plugin` directory contains the  source for your plugins and the
`build` directory will contain the built binary plugins. GymFC will, at
runtime, add the build directory to the Gazebo plugin path so they can be found and loaded.

NOTE! If you are using external plugins create soft links
to each .so file in the build directory.





# Available User Provided Modules

To increase flexability and provide a universal tuning framework, the user must
provide four modules: A flight controller, a flight control tuner, environment
interface, and digital twin. (Note: for neuro-flight controllers typically the
flight controller and tuner are one in the same, e.g., OpenAI baselines) This will expand the flight control research that
can be done with GymFC. For example this opens up the possibilities for tuning
PID gains using optimization strategies such as GAs and PSO. The goal is to provide a collection of open source
modules for users to mix and match. If you have created your own, please let us
know and we will add it below.
 
## Tuners

* [OpenAI baselines](https://github.com/openai/baselines)

## Environments

WIP

## Digital Twins

* [Solo](https://github.com/wil3/gymfc-digitaltwin-solo) Needs help!

## Motor models

* [Element blade theory](https://github.com/wil3/gymfc-aircraft-plugins)


# Custom User Modules

### Digital Twin 


## SDF

Each model.sdf **must** declare the `libAircraftConfigPlugin.so` plugin. 
This is a dummy plugin allowing us to set arbitrary configuration data.
An example configuration may look like this,

```xml
<plugin name="config" filename="libAircraftConfigPlugin.so">
    <!-- Define the total number of motors that shall be controlled -->
    <motorCount>4</motorCount>

    <!-- The center of thrust must be defined in order to attach the aircraft
model to the simulation. The offset will in relation to this specified link -->
    <centerOfThrust> 
        <link>battery</link>
        <offset>0 0 0.058</offset>
    </centerOfThrust>
    <!-- Specify all the sensors this aircraft supports. Valid sensor types 
are "imu, esc, and battery" -->
    <sensors>
      <sensor type="imu">
          <enable_angular_velocity>true</enable_angular_velocity>
          <enable_linear_acceleration>true</enable_linear_acceleration>
          <enable_orientation>true</enable_orientation>
      </sensor>
      <!--
      <sensor type="esc">
            <enable_angular_velocity>true</enable_angular_velocity>
            <enable_temperature>true</enable_temperature>
            <enable_current>true</enable_current>
      </sensor>
      <sensor type="battery">
          <enable_voltage>true</enable_voltage>
          <enable_current>true</enable_current>
      </sensor>
        -->
    </sensors>
</plugin>
```

### API
GymFC communicates with the aircraft through Google Protobuf messages. At a
minimum the aircraft must subscribe to motor commands and publish IMU messages

#### GymFC to Aircraft

*Topic* /aircraft/command/motor 
*Message Type* [MotorCommand.proto]()

#### Aircraft to GymFC

*Topic* /aircraft/sensor/imu 

*Message Type* Imu.proto


*Topic* /aircraft/sensor/esc 

*Message Type* EscSensor.proto


# Development Team 
GymFC was developed and currently maintained by [Wil Koch](https://wfk.io).




