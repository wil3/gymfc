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
logger = logging.getLogger("gymfc")

class PWMPacket:
    def __init__(self, pwm_values, reset=False):
        """ Iniitalize a PWM motor packet 

        Args:
            pwm_values (np.array): an array of PWM values in the range [0, 1000] 
            reset: True if the simulation should be reset
        """
        self.pwm_values = pwm_values
        self.reset = int(reset)

    def encode(self, motor_map = [1, 2, 3, 0]):
        """  Create and return a PWM packet 
        
        Args:
            motor_map (array): 
        """
        scale = 1000.0 # Scale for the digital twin to [0, 1]

        # There is a different motor mapping for the digital twin.
        # This doesnt matter when training, the agent doesn't care,
        # this only matters for when transferring to hardware.
        motor_velocities = [self.pwm_values[motor_map[i]] / scale \
                            for i in range(len(self.pwm_values))]

        # Put the integer first, because of the struct alignment in the 
        # Gazebo plugin. Send this as an int incase we later want to have 
        # this represent other modes we want to control in the simulator
        return struct.pack("<i{}f".format(len(motor_velocities)), self.reset, *motor_velocities)

    def __str__(self):
        return str(self.pwm)

class FDMPacket:

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
        self.motor_velocity = unpacked[17:21]
        self.status_code = unpacked[21]
        unpacked.flags.writeable = False # Sensor values are readonly 
        return self


class ESCClientProtocol:

    def __init__(self):
        """ Initialize the electronic speed controller client """
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
        self.transport.sendto(PWMPacket(motor_values, reset=reset).encode())

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
        self.obs = FDMPacket().decode(data)
    
    def connection_lost(self, exc):
        print("Socket closed, stop the event loop")
        loop = asyncio.get_event_loop()
        loop.stop()

class GazeboEnv(gym.Env):
    MAX_CONNECT_TRIES = 20
    FC_PORT = 9005
    GZ_START_PORT = 11345

    def __init__(self, motor_count=None, world=None, host="localhost"):
        """ Initialize the Gazebo simulation """
        # Init the seed variable
        self.seed()

        self.host = host
        # Search for open ports to allow multile instances of the environment
        # to run in parrellel. Add a nonce to the start port to prevent any
        # conflict of multiple instances usin the same ports during cleanup.
        self.gz_port = self._get_open_port(self.GZ_START_PORT + self.np_random.randint(0,5000))
        self.aircraft_port = self._get_open_port(self.FC_PORT + self.np_random.randint(0,5000))
        self.world = world
        self.pids = []
        self.loop = asyncio.get_event_loop()

        self.stepsize = self.sdf_max_step_size()        
        self.sim_time = 0
        self.last_sim_time = -self.stepsize

        # Set up the action/obs spaces
        state = self.state()
        high = np.array([1.0] * motor_count)
        self.action_space = spaces.Box(-high, high, dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=state.shape, dtype=np.float32) 

        self._start_sim()

        self.sim_stats = {}
        self.sim_stats["steps"] = 0
        self.sim_stats["packets_dropped"] = 0
        self.sim_stats["time_start_seconds"] = time.time()

        # Connect to the Aircraft plugin
        writer = self.loop.create_datagram_endpoint(
            lambda: ESCClientProtocol(),
            remote_addr=(host, self.aircraft_port))
        _, self.esc_protocol = self.loop.run_until_complete(writer) 

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

    def _get_env_var(self, name):
        """ Get an environment variable if it exists 
        
        Args:
            name (string): variable to get
        """
        return os.environ[name] if name in os.environ else ""

    def _start_sim(self):
        """ Start Gazebo """

        signal.signal(signal.SIGINT, self._signal_handler)

        #Port the aircraft reads in
        os.environ["SITL_PORT"] = str(self.aircraft_port)

        # Taken from /usr/share/gazebo-8/setup.sh
        ld_library_path = self._get_env_var("LD_LIBRARY_PATH")
        # if loaded previously
        gz_resource =   self._get_env_var("GAZEBO_RESOURCE_PATH")  
        gz_plugins = self._get_env_var("GAZEBO_PLUGIN_PATH")
        gz_models = self._get_env_var("GAZEBO_MODEL_PATH")

        os.environ["GAZEBO_MASTER_URI"] = "http://{}:{}".format(self.host, self.gz_port)
        os.environ["GAZEBO_MODEL_DATABASE_URI"] = "http://gazebosim.org/models"

        # FIXME Remove hardcoded paths and pull this from somewhere so its 
        # cross platform
        os.environ["GAZEBO_RESOURCE_PATH"] = "/usr/share/gazebo-8" + os.pathsep + gz_resource
        os.environ["GAZEBO_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/gazebo-8/plugins" + os.pathsep + gz_plugins
        os.environ["GAZEBO_MODEL_PATH"] = "/usr/share/gazebo-8/models" + os.pathsep + gz_models

        os.environ["LD_LIBRARY_PATH"] = "/usr/lib/x86_64-linux-gnu/gazebo-8/plugins" + os.pathsep + ld_library_path
        os.environ["OGRE_RESOURCE_PATH"] = "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0"

        # Now load assets
        gz_assets = os.path.join(os.path.dirname(__file__), "assets/gazebo/")
        models = os.path.join(gz_assets, "models")
        plugins = os.path.join(gz_assets, "plugins", "build")
        worlds = os.path.join(gz_assets, "worlds")
        os.environ["GAZEBO_MODEL_PATH"] = "{}:{}".format(models, os.environ["GAZEBO_MODEL_PATH"])
        os.environ["GAZEBO_RESOURCE_PATH"] = "{}:{}".format(worlds, os.environ["GAZEBO_RESOURCE_PATH"])
        os.environ["GAZEBO_PLUGIN_PATH"] = "{}:{}".format(plugins, os.environ["GAZEBO_PLUGIN_PATH"])

        target_world = os.path.join(gz_assets, "worlds", self.world)
        
        p = subprocess.Popen(["gzserver", "--verbose", target_world], shell=False) 
        print ("Starting gzserver process with ID=", p.pid)
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
        self.sim_stats["time_lapse_hours"] = (time.time() - self.sim_stats["time_start_seconds"])/60*60
        self.print_post_simulation_stats()
        self.kill_sim()
        #sys.exit(0)

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

