import gym
from gym import error, spaces, utils
from gym.utils import seeding
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

class PWMPacket:
    def __init__(self, pwm_values, motor_mapping, reset=False):
        """ Initialize a PWM motor packet 

        Args:
            pwm_values (np.array): an array of PWM values in the range [0, 1000] 
            reset: True if the simulation should be reset
        """
        self.pwm_values = pwm_values
        self.motor_mapping = motor_mapping
        self.reset = int(reset)

    def encode(self, motor_map = []):
        """  Create and return a PWM packet 
        
        Args:
            motor_map (array): 
        """
        scale = 1000.0 # Scale for the digital twin to [0, 1]

        # There is a different motor mapping for the digital twin.
        # This doesnt matter when training, the agent doesn't care,
        # this only matters for when transferring to hardware.
        motor_velocities = [self.pwm_values[self.motor_mapping[i]] / scale \
                            for i in range(len(self.pwm_values))]

        # Put the integer first, because of the struct alignment in the 
        # Gazebo plugin. Send this as an int incase we later want to have 
        # this represent other modes we want to control in the simulator
        return struct.pack("<i{}f".format(len(motor_velocities)), self.reset, *motor_velocities)

    def __str__(self):
        return str(self.pwm)

class FDMPacket:
    def __init__(self, motor_mapping):
        self.motor_mapping = motor_mapping

    def decode(self, data):
        unpacked = np.array(list(struct.unpack("<d3d3d4d3d3d4dQ", data)))
        self.timestamp = unpacked[0]
        # All angular velocities from gazebo are in radians/s unless otherwise stated
        self.angular_velocity_rpy = unpacked[1:4]
        self.angular_velocity_rpy[1] *= -1
        self.angular_velocity_rpy[2] *= -1

        self.linear_acceleration_xyz = unpacked[4:7]
        self.orientation_quat = unpacked[7:11]
        self.velocity_xyz = unpacked[11:14]
        self.position_xyz = unpacked[14:17]
        #FIXME move this to the end because its variable
        raw_motor_velocities = unpacked[17:21]
        self.motor_velocity = [raw_motor_velocities[self.motor_mapping[i]] 
                            for i in range(len(raw_motor_velocities))]


        self.status_code = unpacked[21]
        unpacked.flags.writeable = False # Sensor values are readonly 
        return self


class ESCClientProtocol:

    def __init__(self, motor_mapping):
        """ Initialize the electronic speed controller client """
        self.motor_mapping = motor_mapping
        self.obs = None
        self.packet_received = False
        self.exception = None

    def connection_made(self, transport):
        self.transport = transport

    async def write_motor(self, motor_values, reset=False):
        """ Write the motor values to the ESC and then return 
        the current sensor values and an exception if anything bad happend.
        
        Args:
            motor_values (np.array): motor values in range [0, 1000]
            reset (bool): Reset the simulation world
        """
        self.packet_received = False
        self.transport.sendto(PWMPacket(motor_values, self.motor_mapping, reset=reset).encode())

        # Pass the exception back if anything bad happens
        while not self.packet_received:
            if self.exception:
                return (None, self.exception)
            await asyncio.sleep(0.001)

        return (self.obs, None)

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
        self.obs = FDMPacket(self.motor_mapping).decode(data)
    
    def connection_lost(self, exc):
        print("Socket closed, stop the event loop")
        loop = asyncio.get_event_loop()
        loop.stop()

class GazeboEnv(gym.Env):
    MAX_CONNECT_TRIES = 20
    GYMFC_CONFIG_ENV_VAR = "GYMFC_CONFIG"

    def __init__(self):
        # Init the seed variable, user can override this
        self.seed()

        try:
            self.load_config()
        except ConfigLoadException as e:
            raise SystemExit(e)


        # Track process IDs so we can kill em
        self.pids = []
        self.loop = asyncio.get_event_loop()

        self.stepsize = self.sdf_max_step_size()        
        self.sim_time = 0
        self.last_sim_time = -self.stepsize

        # Set up the action/obs spaces
        action_low = np.array([self.output_range[0]] * self.motor_count)
        action_high = np.array([self.output_range[1]] * self.motor_count)
        self.action_space = spaces.Box(action_low, action_high, dtype=np.float32)

        state = self.state()
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=state.shape, dtype=np.float32) 

        # Set up some stats to report at the end, connection are over UDP
        # so it can be useful to see if anything is dropped
        self.sim_stats = {}
        self.sim_stats["steps"] = 0
        self.sim_stats["packets_dropped"] = 0
        self.sim_stats["time_start_seconds"] = time.time()

        # Connect to the Aircraft plugin
        writer = self.loop.create_datagram_endpoint(
            lambda: ESCClientProtocol(self.motor_mapping),
            remote_addr=(self.host, self.aircraft_port))
        _, self.esc_protocol = self.loop.run_until_complete(writer) 

        self._start_sim()

    def load_config(self):
        if self.GYMFC_CONFIG_ENV_VAR not in os.environ:
            message = (
                "Environment variable {} not set. " +
                "Before running the environment please execute, " + 
                "'export {}=path/to/config/file' " +
                "or add the variable to your .bashrc."
            ).format(self.GYMFC_CONFIG_ENV_VAR, self.GYMFC_CONFIG_ENV_VAR)
            raise ConfigLoadException(message)

        config_path = os.environ[self.GYMFC_CONFIG_ENV_VAR]
        if not os.path.isfile(config_path):
            message = "Config file '{}' does not exist.".format(config_path)
            raise ConfigLoadException(message)

        try:
            config = configparser.ConfigParser()
            config.read(config_path)
            common = config["Common"]
            gz = config["Gazebo"]
            ac = config["Aircraft"]
            train = config["Training"]

            # Gazebo specific 
            self.setup_file = gz["SetupFile"]
            self.world = gz["World"]
            self.host = gz["Hostname"]
            self.aircraft_model = gz["AircraftModel"]
            
            # Search for open ports to allow multile instances of the environment
            # to run in parrellel. Add a nonce to the start port to prevent any
            # conflict of multiple instances usin the same ports during cleanup.
            if common.getboolean("UseFixedNetworkPort"):
                self.gz_port =  gz.getint("NetworkPort")
                self.aircraft_port = ac.getint("NetworkPort")
            else:
                self.gz_port = self._get_open_port(
                    self.np_random.randint(
                        *json.loads(gz["NetworkPort"]))
                )
                self.aircraft_port = self._get_open_port(
                    self.np_random.randint(
                        *json.loads(ac["NetworkPort"]))
                )

            # TODO Units
            
            # Training
            self.omega_bounds = json.loads(train["TargetAngularVelocitySamplingRange"])

            # Aircraft
            self.motor_count = ac.getint("MotorCount")
            self.motor_mapping = json.loads(ac["MotorMapping"])
            self.output_range = json.loads(ac["ControlSignalOutputRange"])

        except Exception as e: # Something went wong in the parser
            raise ConfigLoadException(e)

        
    def step_sim(self, action):
        """ Take a single step in the simulator and return the current 
        observations.
         
        Args:
            action (np.array): motor values normalized between [-1:1] in 
        the order [rear_r, front_r, rear_l, font_l]
        """

        return self.loop.run_until_complete(self._step_sim(action))

    async def _step_sim(self, action, reset=False):
        """Complete a single simulation step, return a tuple containing
        the simulation time and the state

        Args:
            action (np.array): motor values normalized between [-1:1] in 
        the order [rear_r, front_r, rear_l, font_l]
        """
        # Convert to motor input to PWM range [0, 1000] to match
        # Betaflight mixer output
        pwm_motor_values = [ ((m + 1) * 500) for m in action]

        # Packets are sent over UDP so they can be dropped, there is no 
        # gaurentee. First we try and send command. If an error occurs in transition 
        # try again or for some reason something goes wrong in the simualator and 
        # the packet wasnt processsed correctly. 
        observations = None
        for i in range(self.MAX_CONNECT_TRIES):
            observations, e = await self.esc_protocol.write_motor(pwm_motor_values, reset=reset)
            if observations:
                if observations.status_code == 1: # Process successful
                    break
                else:
                    print ("Previous command was not processed, resending pwm=", pwm_motor_values)
                    if e:
                        print(e)

            if i == self.MAX_CONNECT_TRIES -1:
                print ("Timeout connecting to Gazebo")
                self.shutdown()
                raise SystemExit("Timeout, could not connect to Gazebo")
            await asyncio.sleep(1)


        # Make these visible
        self.omega_actual = observations.angular_velocity_rpy
        self.sim_time = observations.timestamp 
        # In the event a packet is dropped we could be out of sync. This has only ever been
        # observed when dozens of simulations are run in parellel. We need the speed 
        # of UDP so until this becomes an issue just track how many we suspect were dropped.
        if not np.isclose(self.sim_time, (self.last_sim_time + self.stepsize), 1e-6):
            self.sim_stats["packets_dropped"] += 1

        self.last_sim_time = self.sim_time 
        self.sim_stats["steps"] += 1

        return observations
    
    def _signal_handler(self, signal, frame):
        print("Ctrl+C detected, shutting down gazebo and application")
        self.shutdown()

    def update_env_variables(self, source_file):
	# Source the file in the current environment then use env to dump the 
        # current environment variables (including the ones sourced) then
        # set these variables in the python environment 
        command = ". {}; env".format(source_file)
        pipe = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
        output = pipe.communicate()[0]
        lines = output.splitlines()
        env = {}
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
                    env[last_k] = separated_v
                    separated_v = ""
                env[kv[0]] = kv[1]
                last_k = kv[0]

        # if it happens to be the last key
        if len(separated_v) > 0:
            env[last_k] = separated_v

        os.environ.update(env)

    def _start_sim(self):
        """ Start Gazebo by first updating all the necessary environment
        variables and then starting the Gazebo server"""

        signal.signal(signal.SIGINT, self._signal_handler)

        # Port the aircraft reads in through this environment variable,
        # this is the network channel set up to pass sensor and ESC
        # data back and forth
        os.environ["SITL_PORT"] = str(self.aircraft_port)

        # Source the gazebo setup file to set up vars needed by the simuluator
        self.update_env_variables(self.setup_file)

        # This defines which network port gazebo will start on, we modify 
        # this so we can start multiple instances
        os.environ["GAZEBO_MASTER_URI"] = "http://{}:{}".format(self.host, self.gz_port)

        # Set up paths to our assets
        gz_assets_path = os.path.join(os.path.dirname(__file__), "assets/gazebo/")
        model_path = os.path.join(gz_assets_path, "models")
        plugin_path = os.path.join(gz_assets_path, "plugins", "build")
        world_path = os.path.join(gz_assets_path, "worlds")

        # Add the new paths
        os.environ["GAZEBO_MODEL_PATH"] += (os.pathsep + model_path + os.pathsep
        + self.aircraft_model)
        os.environ["GAZEBO_RESOURCE_PATH"] += os.pathsep + world_path
        os.environ["GAZEBO_PLUGIN_PATH"] += os.pathsep + plugin_path

        target_world = os.path.join(gz_assets_path, "worlds", self.world)
        p = subprocess.Popen(["gzserver", "--verbose", target_world], shell=False) 
        print ("Starting gzserver with process ID=", p.pid)
        self.pids.append(p.pid)


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

        raise Exception("Could not find open port")

    def kill_sim(self):
        """ Kill the gazebo processes based on the original PID  """
        for pid in self.pids:
            p = subprocess.run("kill {}".format(pid), shell=True)
            print("Killing process with ID=", pid)

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
        self.last_sim_time = -self.stepsize
        self.loop.run_until_complete(self._step_sim(self.action_space.low, reset=True))
        self.omega_target = self.sample_target().copy()
        assert np.isclose(self.sim_time, 0.0, 1e-6), "sim time after reset is incorrect, {} ".format(self.sim_time)
        return self.state()

    def close(self):
        self.shutdown()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self):
        raise NotImplementedError

    def state(self):
        raise NotImplementedError

    def sample_target(self):
        raise NotImplementedError

    def render(self, mode='human'):
        p = subprocess.Popen(["gzclient"], shell=False)
        self.pids.append(p.pid)


class SDFNoMaxStepSizeFoundException(Exception):
    pass
class ConfigLoadException(Exception):
    pass

