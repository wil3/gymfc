import asyncio
import struct
import math
import logging
import numpy as np
import os
import os.path
import subprocess
import signal
import sys
import xml.etree.ElementTree as ET
import psutil
import time
import configparser
import json
logger = logging.getLogger("gymfc")
from gymfc.msgs import State_pb2
from gymfc.msgs import Action_pb2
from abc import ABC, abstractmethod
from google.protobuf import descriptor


class ActionPacket:
    def __init__(self, motor, world_control=Action_pb2.Action.STEP):
        """
        Args:
            motor (np.array): an array of motor control signals.
            world_control:
        """
        self.motor = motor
        self.ac = Action_pb2.Action()
        #print ("Sending motor ", motor.tolist())
        self.ac.motor.extend(motor.tolist())
        self.ac.world_control = world_control

    def encode(self):
        """  Encode packet data"""
        msg = self.ac.SerializeToString()
        #print ("Sending ", self.ac, " to Gazebo size=", len(msg), " ",   msg.hex())
        return msg

    def __str__(self):
        return str(self.motor)

class StatePacket:

    def decode(self, data):
        state = State_pb2.State()
        state.ParseFromString(data)
        #print ("State=", state)
        return state

class ActionProtocol:

    def __init__(self):
        """ Initialize the electronic speed controller client """
        self.state_message = None
        self.packet_received = False
        self.exception = None
        self.send_time = None
        self.timeout = 1

    def connection_made(self, transport):
        self.transport = transport

    async def write(self, motor_values, world_control=Action_pb2.Action.STEP):
        """ Write the motor values to the ESC and then return
        the current sensor values and an exception if anything bad happend.

        Args:
            motor_values (np.array): Range dependent on aircraft motor model
        """
        self.packet_received = False
        self.send_time = time.time()
        self.transport.sendto(ActionPacket(motor_values, world_control).encode())

        # Pass the exception back if anything bad happens
        while not self.packet_received:
            if self.exception:
                return (None, self.exception)
            elif time.time() > self.send_time + self.timeout:
                return (None, ReadTimeoutException("Timeout reached waiting for response."))
            await asyncio.sleep(0.001)

        return (self.state_message, None)

    def error_received(self, exc):
        self.exception = exc

    def datagram_received(self, data, addr):
        """ Receive a UDP datagram

        Args:
            data (bytes): raw bytes of packet payload
            addr (string): address of the sender
        """
        # Everything is OK, reset
        self.exception = None
        self.packet_received = True
        self.state_message = StatePacket().decode(data)

    def connection_lost(self, exc):
        print("Socket closed, stop the event loop")
        loop = asyncio.get_event_loop()
        loop.stop()

class FlightControlEnv(ABC):
    """ A generic OpenAI flight control gym environment.

    This class must be extended to implement the task whether for
    attitude or position control. The observations returned by the
    environment are configurable to support a number of different
    aircraft confirations. Actions in the environment consist of
    motor (and/or actuator) control signals defined by a configurable
    range.

    The inherited class must also ensure in the constructor sets the
    OpenAI gym attributes action_space observation_space.
    """


    """ This is the name of the environment variable
    that must be set to the JSON configuration file
    before the environment is created. """
    GYMFC_CONFIG_ENV_VAR = "GYMFC_CONFIG"

    """ Max tries when connecting to Gazebo each
    with a 1 second timeout. """
    MAX_CONNECT_TRIES = 60

    VALID_SENSORS = ["esc", "imu", "battery"]

    def __init__(self, aircraft_config, config_filepath=None, loop=None, verbose=False):
        """ Initialize the simulator

        Args:
            aircraft_config File path of the aircraft Gazebo SDF file
            config_filepath: If provided will override default config.ini file
        """

        self.verbose = verbose
        self.aircraft_sdf_filepath = aircraft_config
        self.enabled_sensor_measurements = []

        # Load the SDF file
        try:
            self.load_config(aircraft_config, config_filepath = config_filepath)
        except ConfigLoadException as e:
            raise SystemExit(e)

        # Track process IDs so we can kill em
        self.process_ids = []
        if not loop:
            self.loop = asyncio.new_event_loop()
        else:
            self.loop = loop

        self.stepsize = self.sdf_max_step_size()
        self.sim_time = 0
        self.last_sim_time = -self.stepsize

        # Set up some stats to report at the end, connection are over UDP
        # so it can be useful to see if anything is dropped
        self.sim_stats = {}
        self.sim_stats["steps"] = 0
        self.sim_stats["packets_dropped"] = 0
        self.sim_stats["time_start_seconds"] = time.time()

        print ("Sending motor control signals to port ", self.aircraft_port)
        # Connect to the Aircraft plugin
        writer = self.loop.create_datagram_endpoint(
            lambda: ActionProtocol(),
            remote_addr=(self.host, self.aircraft_port))
        _, self.ac_protocol = self.loop.run_until_complete(writer)

        self._start_sim()

    def load_config(self, aircraft_config, config_filepath = None):
        """ Load the JSON configuration file defined by the environment
        variable """

        # Priotiry of load, constructor -> environment variable -> default
        current_dir = os.path.dirname(__file__)
        default_config_path = os.path.join(current_dir, "../../gymfc.ini")
        if config_filepath:
            if not os.path.isfile(config_filepath):
                message = "Error, provided configuration file at constructor but not found {}, aborting.".format(config_filepath)
                raise ConfigLoadException(message)
            else:
                config_filepath = config_filepath

        # Load config.ini file specified in environment variable
        elif self.GYMFC_CONFIG_ENV_VAR in os.environ:
            env_config_path = os.environ[self.GYMFC_CONFIG_ENV_VAR]
            if not os.path.isfile(env_config_path):
                message = "Configuration file set by environment varaiable '{}' does not exist.".format(env_config_path)
                raise ConfigLoadException(message)
            else:
                config_filepath = env_config_path
        # Loads default config.ini file
        else:
            if not os.path.isfile(default_config_path):
                message = "Default configuration file at {} missing, aborting.".format(default_config_path)
                raise ConfigLoadException(message)
            else:
                config_filepath = default_config_path

        cfg = configparser.ConfigParser()
        cfg.read(config_filepath)
        default = cfg["DEFAULT"]

        # Gazebo configuration
        # Check that the gazebo setup.ini file specified in the config.ini file is valid
        self.setup_file = default["SetupFile"]
        if not os.path.isfile(self.setup_file):
                message = "Could not find Gazebo setup.sh file at '{}'. Typo?".format(self.setup_file)
                raise ConfigLoadException(message)
        # Save the world to load in Gazebo
        # The worlds that come with this package are empty.world and attitude.world
        self.world = default["World"]
        # Save the gazebo hostname
        self.host = default["Hostname"]

        if cfg.has_option("DEFAULT", "GazeboNetworkPortRangeEnd"):
            self.gz_port = self._get_open_port(
                np.random.randint(
                    default.getint("GazeboNetworkPortRangeBegin"),
                    default.getint("GazeboNetworkPortRangeEnd"))
            )
        else:
            self.gz_port = default.getint("GazeboNetworkPortRangeBegin")
        if cfg.has_option("DEFAULT", "FCPluginPortRangeEnd"):
            self.aircraft_port = self._get_open_port(
                np.random.randint(
                    default.getint("FCPluginPortRangeBegin"),
                    default.getint("FCPluginPortRangeEnd"))
            )
        else:
            self.aircraft_port = default.getint("FCPluginPortRangeBegin")


        # Next, once the config.ini file is parsed, we need to parse the model.sdf file
        self._parse_model_sdf()

    def step_sim(self, ac):
        """ Take a single step in the simulator and return the current
        observations.
        Args:
            ac (np.array): Action to take in the environment bounded by
            output range specificed in config.

        Returns:
            Numpy array defining the environment observations
        """

        return self.loop.run_until_complete(self._step_sim(ac))

    def _flatten_ob(self):
        """ Convert the state packet with observations returned from the digital twin to a single
        1D array that can be used as direct input to the agent.

        Note: Subclass must handle any scaling or normalization

        Returns:
            Numpy array order maintained from aircraft configuration file."""
        ob = []

        #XXX Proto3 doesnt have HasField so we iterate on the enabled
        #sensor measurements from the SDF and maintain order so the agent
        #can index the flatten array
        for key in self.enabled_sensor_measurements:
            field = self.state_message.DESCRIPTOR.fields_by_name[key]
            value = getattr(self.state_message, key)
            if field.label == descriptor.FieldDescriptor.LABEL_REPEATED:
                ob += list(value)
                setattr(self, key, np.array(list(value)))
            else:
                ob += [value]
                setattr(self, key, value)

        return np.array(ob).flatten()

    async def _step_sim(self, ac, world_control=Action_pb2.Action.STEP):
        """Complete a single simulation step, return a tuple containing
        the simulation time and the state

        Args:
            action (np.array): motor values

        Returns:
            Numpy array representing the current observation received from the
            aircraft for the sensor measurements defined in the configuration file.

            Order of the observation values will be preserved according the the order
            they are defined in the configuration file.
        """

        # Packets are sent over UDP so they can be dropped, there is no
        # gaurentee. First we try and send command. If an error occurs in transition
        # try again or for some reason something goes wrong in the simualator and
        # the packet wasnt processsed correctly.
        for i in range(self.MAX_CONNECT_TRIES):
            self.state_message, e = await self.ac_protocol.write(ac, world_control=world_control)
            if self.state_message:
                break
            if i == self.MAX_CONNECT_TRIES -1:
                print ("Timeout communicating with flight control plugin.")
                self.shutdown()
                raise SystemExit("Timeout communicating with flight control plugin.")
            await asyncio.sleep(1)

        # Handle some special cases
        self.sim_time = np.around(self.state_message.sim_time , 3)
        self.force = self.state_message.force
        #self.rate_actual = np.array(list(self.state_message.imu_angular_velocity_rpy))
        #print ("Actual rate=", self.rate_actual)
        # Update the error
        #self.rate_error = self.desired_state() - self.rate_actual

        # In the event a packet is dropped we could be out of sync. This has only ever been
        # observed when dozens of simulations are run in parellel. We need the speed
        # of UDP so until this becomes an issue just track how many we suspect were dropped.
        if not np.isclose(self.sim_time, (self.last_sim_time + self.stepsize), 1e-6):
            self.sim_stats["packets_dropped"] += 1

        self.last_sim_time = self.sim_time
        self.sim_stats["steps"] += 1

        return self._flatten_ob()

    def _signal_handler(self, signal, frame):
        print("Ctrl+C detected, shutting down gazebo and application")
        self.shutdown()

    def update_env_variables(self, source_file, env):
        """ Helper method to source the Gazebo source file to load in all
        of the environment variables used by Gazebo.

        Args:
            source_file (string): Path to the Gazebo source file.
        """
	# Source the file in the current environment then use env to dump the
        # current environment variables (including the ones sourced) then
        # set these variables in the python environment
        command = ". {}; env".format(source_file)
        pipe = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
        output = pipe.communicate()[0]
        lines = output.splitlines()
        gz_env = {}
        separated_v = ""
        last_k = None
        # OK this is super annoying to parse the env vars this way. Vars
        # can have newline characters and any method to return env vars insert
        # new line delimiters so we need to walk through and if a key value
        # pair is separated by new lines combine them when we come across the
        # next valid one
        for i in range(len(lines)):
            line = lines[i].decode("utf-8")
            kv = line.split("=", 1)
            # There was a line break and the new env didnt start
            if len(kv) < 2:
                separated_v += kv[0]
            else:
                # If we start a new var that means if there was
                # a separated value its now the end
                if len(separated_v) > 0:
                    gz_env[last_k] = separated_v
                    separated_v = ""
                gz_env[kv[0]] = kv[1]
                last_k = kv[0]

        # if it happens to be the last key
        if len(separated_v) > 0:
            gz_env[last_k] = separated_v

        env.update(gz_env)

    def _parse_model_sdf(self):

        # Digital twin
        if not os.path.isfile(self.aircraft_sdf_filepath):
            message = "Aircraft SDF file  at location '{}' does not exist.".format(self.aircraft_sdf_filepath)
            raise ConfigLoadException(message)

        # Parse sdf xml
        tree = ET.parse(self.aircraft_sdf_filepath)
        root = tree.getroot()

        # Find the libAircraftConfigPlugin
        # This plugin is required by this module
        els = root.findall(".//plugin[@filename='libAircraftConfigPlugin.so']")
        if len(els) != 1:
            raise SystemExit("Could not find plugin with filename {} from SDF file {} required to load the aircraft model.".format("libAircraftConfigPlugin.so", model_sdf))
        plugin_el = els[0]

        # Get the number of motors defined by the aircraft config plugin
        self.motor_count = int(plugin_el.find("motorCount").text)

        #
        self._get_supported_sensors(plugin_el)


    def _get_supported_sensors(self, plugin_el):
        """ Helper method to get a list of sensors whose data should be published from the aircraft in Gazebo
            Uses the sensors defined in the aircraft config in the sdf file

        Args:
            plugin_el (element): the aircraft config plugin from the sdf xml file
        """

        # Define dictionary that maps from xml tags in the aircraft config in the sdf file to Protobuf messages
        sdf_to_protobuf = {
            "imu": {
                "enable_angular_velocity": "imu_angular_velocity_rpy",
                "enable_linear_acceleration": "imu_linear_acceleration_xyz",
                "enable_orientation":"imu_orientation_quat",
            },
            "esc": {
                "enable_angular_velocity": "esc_motor_angular_velocity",
                "enable_temperature": "esc_temperature",
                "enable_current": "esc_current",
                "enable_voltage": "esc_voltage",
                "enable_force": "esc_force",
                "enable_torque": "esc_torque"
            },
            "battery": {
                "enable_voltage":  "vbat_voltage",
                "enable_current": "vbat_current"
            },
            "position": {
                "enable_gps": "gps"
            }
        }
        # Get all defined sensors
        sensors = plugin_el.find("sensors")
        for sensor in sensors.findall("sensor"):
            # Check if defined sensor type is supported by the simulator
            sensor_type = sensor.attrib["type"]
            if sensor_type in sdf_to_protobuf:
                for enabled in sensor:
                    # Check if specific sensor measurements are enabled for each sensor
                    if enabled.text.lower() == "true":
                        # If enabled, append the Protobuf message name to the list of enabled sensor measurements
                        self.enabled_sensor_measurements.append(sdf_to_protobuf[sensor_type][enabled.tag])
            else:
                raise SystemExit("Unsupported sensor {} found in SDF file".format(sensor_type))

    def _plugins_exist(self, build_path):
        """ Helper to ensure that the libFlightControllerPlugin.so and libAircraftConfigPlugin.so
            exist in the plugin build path

        Args:
            build_path (string): Path to build directory for the plugins
        """
        return (os.path.isfile(os.path.join(build_path, "libFlightControllerPlugin.so")) and
            os.path.isfile(os.path.join(build_path, "libAircraftConfigPlugin.so")))

    def _start_sim(self):
        """ Start Gazebo by first updating all the necessary environment
        variables and then starting the Gazebo server"""


        # XXX
        #signal.signal(signal.SIGINT, self._signal_handler)

        # Port the aircraft reads in through this environment variable,
        # this is the network channel set up to pass sensor and ESC
        # data back and forth
        # XXX Could actual be a race condition if 2+ processes are started
        # at the same time
        container_env = os.environ.copy()

        # Add the SITL port and the path to the sdf file to gazebo container environment variables
        container_env["GYMFC_SITL_PORT"] = str(self.aircraft_port)
        container_env["GYMFC_DIGITAL_TWIN_SDF"] = self.aircraft_sdf_filepath

        # Source the gazebo setup file to set up vars needed by the simuluator
        self.update_env_variables(self.setup_file, container_env)

        # This defines which network port gazebo will start on, we modify
        # this so we can start multiple instances
        container_env["GAZEBO_MASTER_URI"] = "http://{}:{}".format(self.host, self.gz_port)

        # Set up paths to our assets, models, and plugins
        gz_assets_path = os.path.join(os.path.dirname(__file__), "assets/gazebo/")
        model_path = os.path.join(gz_assets_path, "models")
        plugin_path = os.path.join(gz_assets_path, "plugins", "build")

        # Ensure that flight controller and aircraft config plugins are installed
        if not self._plugins_exist(plugin_path):
            readme_path = os.path.join(gz_assets_path, "plugins", "README.md")
            raise SystemExit("Could not find Gazebo plugins. " +
                             "If you have installed in development mode these " +
                             "must be built manually." +
                             " Please refer to {} for manually building the plugins.".format(readme_path))
        # Set up path to worlds
        world_path = os.path.join(gz_assets_path, "worlds")

        # From the gazebo model directory structure the model directory is levels up
        aircraft_model_dir = os.path.abspath(os.path.join(self.aircraft_sdf_filepath, "../../"))

        # Directory for the model plugins
        aircraft_plugin_dir = os.path.abspath(os.path.join(self.aircraft_sdf_filepath, "../plugins/build"))


        # Add the new paths required for Gazebo to load our custom
        # models, plugins and worlds
        container_env["GAZEBO_MODEL_PATH"] += (os.pathsep + model_path + os.pathsep
        + aircraft_model_dir)
        container_env["GAZEBO_RESOURCE_PATH"] += os.pathsep + world_path
        container_env["GAZEBO_PLUGIN_PATH"] += (os.pathsep + plugin_path + os.pathsep +
aircraft_plugin_dir)


        print ("Gazebo Model Path =", container_env["GAZEBO_MODEL_PATH"])
        print ("Gazebo Plugin Path =",container_env["GAZEBO_PLUGIN_PATH"] )

        target_world = os.path.join(gz_assets_path, "worlds", self.world)
        p = None
        # Launch gzserver
        if self.verbose:
            p = subprocess.Popen(["gzserver", "--verbose", target_world], shell=False, env=container_env)
        else:
            p = subprocess.Popen(["gzserver", target_world], shell=False, env=container_env)
        self.env = container_env
        print ("Starting gzserver with process ID=", p.pid)
        self.process_ids.append(p.pid)


    def sdf_max_step_size(self):
        """ Return the max step size read and parsed from the world file"""
        gz_assets = os.path.join(os.path.dirname(__file__), "assets/gazebo/")
        world_filepath = os.path.join(gz_assets, "worlds", self.world)

        tree = ET.parse(world_filepath)
        root = tree.getroot()
        els = root.findall("./world/physics/max_step_size")
        if len(els) == 0:
            raise SDFNoMaxStepSizeFoundException()

        return float(els[0].text)

    def _get_open_port(self, start_port):
        """ Return an available open port, starting from start_port

        Args:
            start_port (int): first port to try, will increment until port is open
        """

        connections = psutil.net_connections()
        open_ports = []
        for c in connections:
            open_ports.append(c.laddr.port)
        for port in range(start_port, 2**16):
            if not (port in open_ports):
                return port

        raise Exception("Could not find an open port")

    def kill_sim(self):
        """ Kill the gazebo processes based on the original PID  """
        for pid in self.process_ids:
            p = subprocess.run("kill {}".format(pid), shell=True)
            print("Killing Gazebo process with ID=", pid)

    def print_post_simulation_stats(self):
        print ("\nSimulation Stats")
        print ("-----------------")
        key_len = max(list(map(len,list(self.sim_stats.keys())))) + 5
        for key, values in self.sim_stats.items():
            print("{0: <{fill}}{1}".format(key, values, fill=key_len))
        print ("\n")

    def shutdown(self):
        self.sim_stats["time_lapse_hours"] = (time.time() - self.sim_stats["time_start_seconds"])/(60*60)
        self.print_post_simulation_stats()
        self.kill_sim()

    def reset(self):
        """ Reset the environment (compatible with OpenAI API).

        Warning: When inheriting this class you will most likely need to override
        this method and call super to reset any internal state used in the child
        class. """
        # XXX Because we use asyncio if a sleep is done right after a command
        # its possible well miss the recieve message
        self.last_sim_time = -self.stepsize
        # Motor values are ignored during a reset so just send whatever
        ob = self.loop.run_until_complete(self._step_sim(np.zeros(self.motor_count), world_control=Action_pb2.Action.RESET))
        assert np.isclose(self.sim_time, 0.0, 1e-6), "sim time after reset is incorrect, {} ".format(self.sim_time)
        return ob

    def close(self):
        self.shutdown()

    def render(self, mode='human'):
        """ Launch the Gazebo client """
        p = subprocess.Popen(["gzclient"], shell=False, env=self.env)
        self.process_ids.append(p.pid)


# Exceptions
class SDFNoMaxStepSizeFoundException(Exception):
    pass

class ConfigLoadException(Exception):
    pass

class ReadTimeoutException(Exception):
    pass
