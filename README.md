# GymFC

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
## Table of contents

* Installation
* Features
* News
* Development Team
* Contributions




# Features

* Support for IMU, ESC and battery sensors
* Aircraft agnostic - support for any type of aircraft just configure number of
  actuators and sensors.
* Digital twin independence - digital twin is developed external to GymFC
  allowing separate versioning.
* Google protobuf aircraft digital twin API for publishing control
  signals and subscribing to sensor data. 
* Flexible agent interface allowing controller development for any type of flight control systems.
* Compatible with OpenAI environments.
* Support for Gazebo 8, 9, and 11. Gazebo plugins are built dynamically depending on
  your installed version. 
* Configurable through JSON


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
2. From root directory of this project, `pip3 install .`
3. Confirm `SetupFile` in `gymfc.ini` is pointing to the correct location.


# Development 

If you plan to work with the GymFC source code you will want to install it in
development mode, `pip3 install -e .` from the root directory. You will also
need to build the plugin manually by running the script
`gymfc/envs/assets/gazebo/plugins/build_plugin.sh`. 


## Digital Twin 
For simplicity the GymFC environment takes as input the aircraft `model.sdf` which contains all properties for the
aircaft in a single location. The SDF declares all the visualizations, geometries and plugins for the aircraft.

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
runtime, add the build directory to the Gazebo plugin path.

If you are using external plugins (e.g.,[gymfc-aircraft-plugins](https://github.com/wil3/gymfc-aircraft-plugins) ) create soft links
to each .so file in the build directory.



### SDF

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

## Agent Interface


# Development Team and Contributions
GymFC was developed and currently maintained by [Wil Koch](https://wfk.io).

