![GymFC](https://github.com/wil3/gymfc/blob/master/images/gymfc-logo.png)
[![Build Status](https://travis-ci.com/wil3/gymfc.svg?branch=master)](https://travis-ci.com/wil3/gymfc)
[![All-Contributors](https://img.shields.io/badge/dynamic/json?color=orange&label=all%20contributors&query=%24.contributors.length&url=https%3A%2F%2Fraw.githubusercontent.com%2Fwil3%2Fgymfc%2Fmaster%2F.all-contributorsrc)](#contributors-)

GymFC is flight control tuning framework with a focus in attitude control. 
GymFC was first introduced in the [manuscript](http://wfk.io/docs/gymfc.pdf) "Reinforcement learning for UAV attitude control" in which a simulator was used to
synthesize neuro-flight attitude controllers that exceeded the performance of a traditional PID controller. 
Since the projects initial release it has matured to become a modular
framework
for tuning flight control systems, not only for synthesizing neuro-flight
controllers but also tuning traditional controllers as well. 
GymFC is the primary method for developing controllers to be used in the worlds
first neural network supported
flight control firmware [Neuroflight](https://wfk.io/neuroflight). 
Details of the project and its architecture are best described in Wil Koch's 
[thesis](http://wfk.io/docs/WilliamKochThesisFINAL.pdf) "Flight Controller Synthesis Via Deep Reinforcement Learning".

Please use the following BibTex entries to cite our work,
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
```
@article{koch2019flight,
  title={Flight Controller Synthesis Via Deep Reinforcement Learning},
  author={Koch, William},
  journal={arXiv preprint arXiv:1909.06493},
  year={2019}
}
```

![Architecture](https://github.com/wil3/gymfc/blob/master/images/gymfc2-arch.png)


## Table of contents

* [Features](https://github.com/wil3/gymfc#features)
* [News](https://github.com/wil3/gymfc#news)
* [Installation](https://github.com/wil3/gymfc#installation)
* [Getting Started](https://github.com/wil3/gymfc#getting-started)
* [User Modules](https://github.com/wil3/gymfc#available-user-provided-modules)
* [Custom Modules](https://github.com/wil3/gymfc#custom-user-modules)
* [Examples](https://github.com/wil3/gymfc#examples)
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
* May 2020 - [NF1](https://rotorbuilds.com/build/15163) quadcopter model and reward functions used in this thesis work is published in the `examples` directory.
* August 2019 - GymFC synthesizes neuro-controller with [new level of
  performance](https://www.youtube.com/watch?v=MByCyEnsYP0).  
* August 2019 - Thesis is defended, [Flight Controller Synthesis via Deep
  Reinforcement Learning](http://wfk.io/docs/WilliamKochThesisFINAL.pdf). 
* July 2019 - GymFC v0.2.0 is released. 
* December 2018 - Our GymFC manuscript is accepted to the journal ACM Transactions on Cyber-Physical Systems.
* November 2018 - Flight controller synthesized with GymFC achieves stable
  flight in [Neuroflight](https://github.com/wil3/neuroflight).
* September 2018 - GymFC v0.1.0 is released.
* April 2018 - Pre-print of our paper is published to [arXiv](https://arxiv.org/abs/1804.04154). 


# Installation 

## Quick start
To install GymFC and its dependencies on Ubuntu 18.04 execute,
```
sudo MAKE_FLAGS=-j4 ./install_dependencies.sh
pip3 install .
```

## Dependencies
GymFC runs on Ubuntu 18.04 and uses  [Gazebo v10.1.0](http://gazebosim.org/tutorials?tut=install_from_source&cat=install) with [Dart v6.7.0](https://github.com/dartsim/dart/tree/v6.7.0) for the backend simulator. To use Dart with Gazebo, they must be installed from source. For why Gazebo must be used with Dart see this [video](https://www.youtube.com/watch?v=d3NyFU0bVT0). The easiest way to install the dependencies is with the provided `install_dependencies.sh` script. By default it will run `make` with a single job. You can override the `make` flags with the `MAKE_FLAGS` environment variable. Building Gazebo from source is very resource intensive. If you have sufficient memory increase the number of jobs to run in parallel. For example to run four jobs in parallel execute, 
```
sudo MAKE_FLAGS=-j4 ./install_dependencies.sh
```
Note, this script may take more than an hour to execute. If your build fails
check `dmesg` but the most common reason will be out-of-memory failures. 

## GymFC
(Optional) It is suggested to set up a [virtual environment](https://docs.python.org/3/library/venv.html) to install GymFC into. From the project root run,
   `python3 -m venv env`. This will create an environment named `env` which
will be ignored by git. To enable the virtual environment, `source
env/bin/activate` and to deactivate, `deactivate`.  

Install GymFC,
```
pip3 install .
```
This will install the Python dependencies and also build the Gazebo plugins and
messages. 

## Developing with GymFC
If you plan to modify the GymFC code you will need to install in
edit/development mode. 
```
pip3 install -e .
```
You will also have to manually install the Gazebo plugins by executing,
```
gymfc/envs/assets/gazebo/plugins/build_plugin.sh
```
If you deviate from this installation instructions (e.g., installing Gazebo in
a different location other than specific in `install_dependencies.sh`), you
may need to change the location of the Gazebo `setup.sh` defined by the
variable SetupFile in `gymfc/gymfc.ini`.

## Verifying install

GymFC requires an aircraft model (digital twin) to run. The NF1 racing
quadcopter model is available in `examples/gymfc_nf/twins/nf1` if you need a
model for testing. To test everything is installed correctly run,

```
python3 tests/test_start_sim.py --verbose examples/gymfc_nf/twins/nf1/model.sdf
```
If everything is OK you should see the NF1 quadcopter model in Gazebo.

You will see the following error message because you have not built the
motor and IMU plugins yet.
```
[Err] [Plugin.hh:187] Failed to load plugin libgazebo_motor_model.so: libgazebo_motor_model.so: cannot open shared object file: No such file or directory
[Err] [Plugin.hh:187] Failed to load plugin libgazebo_imu_plugin.so: libgazebo_imu_plugin.so: cannot open shared object file: No such file or directory
```
Also the following error message is normal,
``` 
[Err] [DARTJoint.cc:195] DARTJoint: SetAnchor is not implemented
```

To use the NF1 model for further testing read examples/README.md. 

## Install by Docker
This repository includes an experimental docker build in `docker/demo` that demos the usage of GymFC. 
It has been tested on MacOS 10.14.3 and Ubuntu 18.04, however the Gazebo client
has not been verified to work for Ubuntu. This docker image can help ensure you
are running a supported environment for GymFC.

### Install dependencies 

For Mac, install [Docker for Mac](https://docs.docker.com/docker-for-mac/install/) and [XQuartz](https://www.xquartz.org/) on your system.
For Ubuntu, install [Docker for Ubuntu](https://docs.docker.com/engine/install/ubuntu/).

### Build and test 
Build the docker image

```bash
docker build  . -t gymfc:demo
```
This will take a while as it compiles mesa drivers, gazebo and dart. It is recommended to give Docker a large part of the host's resources.
All incoming connections will forward to xquartz:

```bash
xhost +
```

Example usage, run the image and test test_step_sim.py using the [Solo digital twin](https://github.com/wil3/gymfc-digitaltwin-solo.git),
```bash
docker run -ti -e DISPLAY=<hostip>:0 \
-v <path-to-gymfc-digitaltwin-solo>/models/solo/model.sdf:/gymfc/demo/models/solo/model.sdf \
gymfc:demo \
"python3 /gymfc/tests/test_step_sim.py --gymfc-config /gymfc/gymfc.ini --verbose /gymfc/demo/models/solo/model.sdf  1 1 1 1"
```

Replace _<hostip>_ by the external ip of your system to allow gymfc to connect to your XQuartz server and _<path-to-gymfc-digitaltwin-solo>_ to where you cloned the Solo repo.
Take special note that the `test_step_sim.py` parameters are using the containers
path, not the host's path.


# Getting Started 

The simplest environment can be created with,

```python
from gymfc.envs.fc_env import FlightControlEnv
class MyEnv(FlightControlEnv):
    def __init__(self, aircraft_config, config=None, verbose=False):
        super().__init__(aircraft_config, config_filepath=config, verbose=verbose)
```

By inheriting FlightControlEnv you now have access to the `step_sim` and
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

To increase flexibility and provide a universal tuning framework, the user must
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

* [gymfc_nf-step-v1](https://github.com/wil3/gymfc/tree/master/examples/gymfc_nf/envs)

## Digital Twins

* [NF1](https://github.com/wil3/gymfc/tree/master/examples/gymfc_nf/twins/nf1)
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

# Examples

The OpenAI environment and digital twin models used in Wil Koch's thesis can be found in the
`examples/` directory. 

# Creator
[Wil Koch](https://wfk.io)

# Contributors ✨

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tr>
    <td align="center"><a href="https://wfk.io"><img src="https://avatars3.githubusercontent.com/u/4587660?v=4" width="100px;" alt=""/><br /><sub><b>Wil Koch</b></sub></a><br /><a href="https://github.com/wil3/gymfc/commits?author=wil3" title="Code">💻</a> <a href="https://github.com/wil3/gymfc/commits?author=wil3" title="Documentation">📖</a> <a href="#example-wil3" title="Examples">💡</a> <a href="#infra-wil3" title="Infrastructure (Hosting, Build-Tools, etc)">🚇</a></td>
    <td align="center"><a href="https://github.com/MichaelManz"><img src="https://avatars0.githubusercontent.com/u/1280036?v=4" width="100px;" alt=""/><br /><sub><b>Michael Friedrich</b></sub></a><br /><a href="https://github.com/wil3/gymfc/commits?author=MichaelManz" title="Code">💻</a> <a href="#example-MichaelManz" title="Examples">💡</a> <a href="#infra-MichaelManz" title="Infrastructure (Hosting, Build-Tools, etc)">🚇</a></td>
    <td align="center"><a href="https://github.com/sidmysore"><img src="https://avatars2.githubusercontent.com/u/12174271?v=4" width="100px;" alt=""/><br /><sub><b>Sid Mysore</b></sub></a><br /><a href="https://github.com/wil3/gymfc/issues?q=author%3Asidmysore" title="Bug reports">🐛</a></td>
    <td align="center"><a href="https://github.com/varunag18"><img src="https://avatars0.githubusercontent.com/u/1232584?v=4" width="100px;" alt=""/><br /><sub><b>varunag18</b></sub></a><br /><a href="https://github.com/wil3/gymfc/commits?author=varunag18" title="Tests">⚠️</a></td>
    <td align="center"><a href="https://github.com/xabierolaz"><img src="https://avatars0.githubusercontent.com/u/31626226?v=4" width="100px;" alt=""/><br /><sub><b>xabierolaz</b></sub></a><br /><a href="https://github.com/wil3/gymfc/commits?author=xabierolaz" title="Tests">⚠️</a></td>
    <td align="center"><a href="https://github.com/SwapnilPande"><img src="https://avatars3.githubusercontent.com/u/9200948?v=4" width="100px;" alt=""/><br /><sub><b>Swapnil Pande</b></sub></a><br /><a href="https://github.com/wil3/gymfc/issues?q=author%3ASwapnilPande" title="Bug reports">🐛</a> <a href="#ideas-SwapnilPande" title="Ideas, Planning, & Feedback">🤔</a> <a href="https://github.com/wil3/gymfc/commits?author=SwapnilPande" title="Tests">⚠️</a></td>
  </tr>
</table>

<!-- markdownlint-enable -->
<!-- prettier-ignore-end -->
<!-- ALL-CONTRIBUTORS-LIST:END -->

Want to become a contributor?! Visit [CONTRIBUTING.md](https://github.com/wil3/gymfc/blob/master/CONTRIBUTING.md) for more information to get started. 
