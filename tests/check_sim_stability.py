import argparse
import os
from gymfc.envs.fc_env import FlightControlEnv 
import numpy as np
import time
import asyncio
import pygazebo
import xml.etree.ElementTree as ET
from threading import Thread

from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import cm


RUNNING = True

class GazeboMessageHandler:
    """
    General idea is that the distance between all the aircraft subcompoents should be the same.

    Thus given a undirected graph we can take a snapshot of all the distances  (ie length of the edges) 
    """
    def __init__(self, logger, loop, host, port, model_name, threshold=1e-6):
        """
        Initialize the data capturer

        Args:
            loop:
            host:
            port:
            model_name
            threshold: The distance threshold in meters that needs to be greater
            to consider the model instable. 
        """
        self.logger = logger
        self.loop = loop
        self.host = host
        self.port = port
        self.threshold = threshold
        self.pose_stamped = None
        self.e = []
        self.model_name = model_name
        self.submodel_names = []

    async def connect(self):
        try: 
            global RUNNING
            # Connect to Gazebo
            for i in range(10):
                try:
                    self.manager = await pygazebo.connect((self.host, self.port))
                    print ("Pygazebo connected to Gazebo!")
                    break
                except Exception as e:
                    print("Attempt {}: Exception occured connecting to Gazebo. Reason: {}".format(i+1, e))
                await asyncio.sleep(1)
                
            self.pose_subscriber = self.manager.subscribe('/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', self.pose_callback)

            # Wait until we get the first one
            #while not self.pose_stamped:
            await self.pose_subscriber.wait_for_connection()
            while RUNNING:
                await asyncio.sleep(0.1)
        except Exception as e:
            print ("Erorr occurred ", e)

    def compute_edges(self, points):
        
        # n x n matrix, diagonal = 0, symmetric
        # Compute all edgets
        #print ("Points=", points)
        #edges = []
        edges = np.zeros([len(points), len(points)])

        for i in range(len(points)):
            for j in range(len(points)):
                if j > i:
                    d = np.linalg.norm(points[i] - points[j])
                    #edges.append(d)
                    edges[i][j] = d
                    
        #print ("Edges=", edges)
        return edges


    def pose_callback(self, data):
        """ Callback for the all the model pose data

        NOTE! The publish rate is hardcoded in gazebo, to find the 
        publish rate of a topic use the gz command as follows when a Gazebo
        instances is running,
        gz topic -z /gazebo/default/pose/info
        I measured it to be around 60Hz
        """
        # Sending of the data is at a fixed rate hardcoded in Gazebo
        self.pose_stamped = pygazebo.msg.v11.poses_stamped_pb2.PosesStamped()
        self.pose_stamped.ParseFromString(data)

        t = np.around(self.pose_stamped.time.sec + (self.pose_stamped.time.nsec  * 1e-9) , 3)
        prefix = self.model_name + "::" # Sub components
        points = []
        names = []
        for pose in self.pose_stamped.pose:
            if pose.name.startswith(prefix):
                points.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
                names.append(pose.name)


        # Init the names, assume modeles don't change
        if len(self.submodel_names) == 0:
            self.submodel_names = names

        e = self.compute_edges(points) 
        if len(e) > 0:
            if len(self.e) > 0:
                diff = np.abs(self.e - e)
                total_diff = np.sum(diff)
                #print ("Diff Sum=", total_diff)
                #print ("Diff=", diff)
                instabilities = diff > self.threshold
                for i in range(len(instabilities)):
                    for j in range(len(instabilities)):
                        if j > i:
                            if instabilities[i][j]:
                                #print ("ERROR model instable  between {} and {}".format(self.submodel_names[i], self.submodel_names[j]))
                                pass

                self.logger.add_stability([t, total_diff])
            else:
                # init
                self.e = np.copy(e)

        #print (self.pose_stamped)

    def start_loop(self, loop):
        print ("Starting capture loop")

        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.connect())


def step_sim(env, logger):
    """ Evaluate an environment with the given policy """

    global RUNNING
    step_size = env.stepsize
    logger.step_size = step_size
    logger.sim_step_size = step_size
    sim_steps = env.max_sim_time * (1/step_size)
    motor_inc = 1/sim_steps
    # Try each axis doing a ramp
    num_motor_steps = 1000.0

    motor_commands = [
        np.array([1, 0, 0, 0]), # roll right
        np.array([0, 1, 0, 0]), # roll right
        np.array([0, 0, 1, 0]), # roll right
        np.array([0, 0, 0, 1]), # roll right

        np.array([1, 1, 0, 0]), # roll right
        np.array([0, 0, 1, 1]), # roll left 
        np.array([1, 0, 1, 0]), # pitch forward 
        np.array([0, 1, 0, 1]), # pitch backwards 
        np.array([1, 0, 0, 1]), # Yaw CCW 
        np.array([0, 1, 1, 0]),  # Yaw CW

        np.array([0, 1, 1, 1]),  # Yaw CW
        np.array([1, 0, 1, 1]),  # Yaw CW
        np.array([1, 1, 0, 1]),  # Yaw CW
        np.array([1, 1, 1, 0])  # Yaw CW
    ]

    for m in motor_commands:

        print ("Target motor=", m)
        motor_signals = np.zeros(4)
        ob = env.reset()
        while True:
            #print (motor_signals)
            ob = env.step_sim(motor_signals)
            #print ("RPY", env.imu_angular_velocity_rpy, " M=", motor_signals)
            #if delay > 0:
            # XXX Measured the publish rate at 60Hz to be on the safe size
            # send commands at 10Hz
            time.sleep(0.1)
            if env.is_done():
                break
            motor_signals = motor_signals + (m * np.array([motor_inc]*4))

            logger.add_aircraft_io([env.sim_time] + env.imu_angular_velocity_rpy.tolist())

        logger.trial_index += 1

    RUNNING = False
    print ("Closing env")
    env.close()

def step_loop(args, logger):
    loop = asyncio.new_event_loop()
    env = Sim(args.aircraftconfig, config_filepath=args.gymfc_config, max_sim_time=args.max_sim_time, loop = loop, verbose=False)
    if args.render:
        env.render()
    step_sim(env, logger)

#def start(loop, env, capture):
def start(env, capture):
    new_loop = asyncio.new_event_loop()
    t = Thread(target=step_loop, args=(new_loop, env))
    t.start()

    #loop = asyncio.get_event_loop()
    loop = asyncio.new_event_loop()
    t2 = Thread(target=capture.start_loop, args=(loop,))
    #t2.start()
    #futures = asyncio.gather(capture.connect())
    #loop.run_until_complete(capture.connect())


class Sim(FlightControlEnv):

    def __init__(self, aircraft_config, config_filepath=None, max_sim_time=1,
                 loop=None, verbose=True):
        super().__init__(aircraft_config, config_filepath=config_filepath,
                         loop=loop, verbose=verbose)
        self.max_sim_time = max_sim_time

    def is_done(self):
        return self.sim_time > self.max_sim_time

class DataLogger:
    """ Class to store all the results, resources aren't shared so 
    there won't be any synch issues """
    def __init__(self):
        self.trial_index = 0 # Only stepper can change this value

        self.data_ac = []
        self.data_pose = []

        self.step_size = 0
        self.physics_type = None

        # Use this to normalize
        self.min_d_sum = 1e6 
        self.max_d_sum = 0
        self.min_rate = np.array([1e6]*3)

    def add_aircraft_io(self, data):
        if len(self.data_ac) <= self.trial_index:
            self.data_ac.append([data])
        else:
            self.data_ac[self.trial_index].append(data)

    def add_stability(self, data):
        """ Data where data is an array with elements t and the sum of the diffs"""
        d_sum = data[1]
        if self.min_d_sum > d_sum:
            self.min_d_sum = d_sum 
        if self.max_d_sum < d_sum:
            self.max_d_sum = d_sum

        if len(self.data_pose) <= self.trial_index:
            self.data_pose.append([data])
        else:
            self.data_pose[self.trial_index].append(data)

    def process_results(self):
        """ Sync the results up accoding to the sim time and determine
        which rates are stable and instable in the sim 
        Returns Array or RPYs, Max rate that is stable, color for each RPY point, """
        threshold = 0.001 #1mm
        instable = []
        stable = []

        d_range = self.max_d_sum - self.min_d_sum
        print ("Min=", self.min_d_sum)
        print ("Max=", self.max_d_sum)
        print ("D range=", d_range)


        norm = matplotlib.colors.Normalize(vmin=self.min_d_sum, vmax=self.max_d_sum, clip=True)
        mapper = cm.ScalarMappable(norm=norm, cmap=cm.plasma)

        #print (self.data_pose)
        max_r = 0
        colors = []
        rates = []
        ds = [] #array of distance sums

        for i in range(len(self.data_pose)):

            ac_trial = np.array(self.data_ac[i])
            for pose in self.data_pose[i]:
                t = pose[0]
                d_sum = pose[1]
                found_row = ac_trial[np.where(ac_trial[:,0] == t)]
                if len(found_row) > 0:
                    rate = found_row[0][1:]
                    #print ("t=", t, " d_sum", d_sum)
                    #print (found_row)

                    colors.append(mapper.to_rgba(d_sum))
                    rates.append(rate)
                    ds.append(d_sum)

                    if d_sum >= threshold:
                        instable.append(rate)
                        if (rate < self.min_rate).all():
                            self.min_rate = rate.copy()
                    else:
                        stable.append(rate)
                        r = np.linalg.norm(rate)
                        if r > max_r:
                            max_r = r
            #print ("t=", t, " rpy=", rate)
        #return np.array(instable), np.array(stable), max_r, colors
        print ("Instability occurs at ", self.min_rate)
        np.savetxt("/tmp/unstable.txt", instable )
        return np.array(rates), max_r, colors, ds

    def plot(self):

        #instable, stable, r, colors = self.process_results()
        rates, r, colors, ds = self.process_results()

        #print ("Instable", instable)
        #print ("Stable", stable)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        #stable = ax.scatter(data[:,0], data[:,1], data[:,2], c=colors, label=labels)
        """
        if len(stable)>0:
            ax_stable = ax.scatter(stable[:,0], stable[:,1], stable[:,2], c='b')
        if len(instable)>0:
            ax_instable = ax.scatter(instable[:,0], instable[:,1], instable[:,2], c='r')
        """
        ax_instable = ax.scatter(rates[:,0], rates[:,1], rates[:,2], c=ds, cmap='plasma')

        steps = 20
        theta, phi = np.linspace(0, 2 * np.pi, steps), np.linspace(0, np.pi, steps)
        THETA, PHI = np.meshgrid(theta, phi)
        x = r * np.sin(PHI) * np.cos(THETA)
        y = r * np.sin(PHI) * np.sin(THETA)
        z = r * np.cos(PHI)
        #ax.plot_wireframe(x, y, z, color="b")


        ax.set_xlabel('Roll (deg/s)')
        ax.set_ylabel('Pitch (deg/s)')
        ax.set_zlabel('Yaw (deg/s)')

        """
        if len(instable)>0:
            print ("HERE")
            ax.legend((ax_stable, ax_instable), ("Stable Region", "Instable"))
        else:
            print ("HERE2")
            ax.legend((ax_stable,), ("Stable Region",))
        """
        title_mapping = {
            "dart" : "DART",
            "ode" : "ODE",
            "bullet" : "Bullet",
            "simbody" : "Simbody"
        }
        #plt.title("{} Physics Engine - Step size {}".format(title_mapping[self.physics_type], self.step_size))

        #_data = plt.cm.jet()

        cb = plt.colorbar(ax_instable, ax=ax)

        cb.set_label(label='Model Drift (meters)', weight='bold')
        plt.show()

def get_physics_type_from_world():
    """ Return the max step size read and parsed from the world file"""
    gz_assets = os.path.join(os.path.dirname(__file__), "../gymfc/envs/assets/gazebo/")
    world_filepath = os.path.abspath(os.path.join(gz_assets, "worlds", "attitude.world"))
    tree = ET.parse(world_filepath)
    root = tree.getroot()
    el = tree.findall(".//physics")[0]
    return el.attrib["type"]

def get_model_name_from_sdf(sdf):
    """ Return the model name from the given SDF """
    tree = ET.parse(sdf)
    root = tree.getroot()
    el = root.find("model")
    return el.attrib["name"]
    

if __name__ == "__main__":

    parser = argparse.ArgumentParser("Test the stability of the given model and simulator.")
    parser.add_argument('aircraftconfig', help="File path of the aircraft SDF.")
    parser.add_argument('--gymfc-config', default=None, help="Option to override default GymFC configuration location.")
    parser.add_argument('--delay', default=0, type=float, help="Second delay betwee steps for debugging purposes.")
    parser.add_argument('--max-sim-time', default=1, type=float, help="Time in seconds the sim should run for.")
    parser.add_argument('--gazebo-version', default=8, type=int, help="The Gazebo version you are using.")
    parser.add_argument('--gazebo-port', default=11345, type=int, help="")
    parser.add_argument('--gazebo-host', default="localhost", type=str, help="")
    parser.add_argument('--render', action="store_true", help="Launch Gazebo GUI")

    args = parser.parse_args()

    # TODO dynamically load depending on version in setup.ini
    if args.gazebo_version == 8:
        pass
    elif args.gazebo_version == 9:
        pass
    elif args.gazebo_version == 10:
        pass
    elif args.gazebo_version == 11:
        import pygazebo.msg.v11.poses_stamped_pb2
    else:
        raise SystemExit("Unsupported Gazebo version {}".format(args.gazebo_version))

    model_name = get_model_name_from_sdf(args.aircraftconfig)

    logger = DataLogger()
    logger.physics_type = get_physics_type_from_world()

    capture = GazeboMessageHandler(logger, None, args.gazebo_host, args.gazebo_port, model_name=model_name)

    t = Thread(target=step_loop, args=(args,logger, ))
    t.start()

    loop = asyncio.new_event_loop()
    t2 = Thread(target=capture.start_loop, args=(loop,))
    t2.start()

    t.join()
    t2.join()
    
    print ("threads joined, plotting")
    logger.plot()
