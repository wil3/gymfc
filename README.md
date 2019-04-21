# GymFC

GymFC is a simulation environment for developing and testing flight control systems with a focus in attitude control. 
Intially GymFC was first introduced in the [manuscript]() "Reinforcement learning for UAV attitude control" in which the simulator was used to
synthesize neuro-flight attitude controllers that exceeded the performance of a traditional PID controller. 
Since the intial release of the project is has matured, and become more modular
to support a wide range of flight control system and aircraft.
Currently, GymFC is the primary method for developing controllers to be used in the worlds
first neural network based
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
* Support for Gazebo 8, 9, and 11. Gazebo plugins are built dynamically depending on
  your installed version. 
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

Note, Ubuntu 16.04 LTS and 18.04 LTS are the only OS currently supported. Please submit a PR for the
README.md if you are
able to get it working on another platform.   
1. Install [Gazebo 9](http://gazebosim.org/download) with `sudo
   apt-get install gazebo9.`. Then the dev package required for building the plugins, `sudo apt-get install libgazebo9-dev`.
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
GymFC takes as input an aircraft model.sdf defining all the details for the
aircaft. The SDF delares plugins implementing GymFC's aircraft API to
communicate to and from the aircraft.

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
are "imu, esc, and battery">
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

#### GymFC -> Aircraft

*Topic* /aircraft/command/motor 
*Message Type* [MotorCommand.proto]()

#### Aircraft -> GymFC

*Topic* /aircraft/sensor/imu 
*Message Type* Imu.proto

*Topic* /aircraft/sensor/esc 
*Message Type* EscSensor.proto

## Agent Interface

