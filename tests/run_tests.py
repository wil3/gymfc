import argparse
import gym
import gymfc
import matplotlib.pyplot as plt
import numpy as np
from mpi4py import MPI
import math
import os

def plot_step_response(desired, actual,
                 end=1., title=None,
                 step_size=0.001, threshold_percent=0.1):
    """
        Args:
            threshold (float): Percent of the start error
    """

    #actual = actual[:,:end,:]
    end_time = len(desired) * step_size
    t = np.arange(0, end_time, step_size)

    #desired = desired[:end]
    threshold = threshold_percent * desired

    plot_min = -math.radians(350)
    plot_max = math.radians(350)

    subplot_index = 3
    num_subplots = 3

    f, ax = plt.subplots(num_subplots, sharex=True, sharey=False)
    f.set_size_inches(10, 5)
    if title:
        plt.suptitle(title)
    ax[0].set_xlim([0, end_time])
    res_linewidth = 2
    linestyles = ["c", "m", "b", "g"]
    reflinestyle = "k--"
    error_linestyle = "r--"

    # Always
    ax[0].set_ylabel("Roll (rad/s)")
    ax[1].set_ylabel("Pitch (rad/s)")
    ax[2].set_ylabel("Yaw (rad/s)")

    ax[-1].set_xlabel("Time (s)")


    """ ROLL """
    # Highlight the starting x axis
    ax[0].axhline(0, color="#AAAAAA")
    ax[0].plot(t, desired[:,0], reflinestyle)
    ax[0].plot(t, desired[:,0] -  threshold[:,0] , error_linestyle, alpha=0.5)
    ax[0].plot(t, desired[:,0] +  threshold[:,0] , error_linestyle, alpha=0.5)
 
    r = actual[:,0]
    ax[0].plot(t[:len(r)], r, linewidth=res_linewidth)

    ax[0].grid(True)



    """ PITCH """

    ax[1].axhline(0, color="#AAAAAA")
    ax[1].plot(t, desired[:,1], reflinestyle)
    ax[1].plot(t, desired[:,1] -  threshold[:,1] , error_linestyle, alpha=0.5)
    ax[1].plot(t, desired[:,1] +  threshold[:,1] , error_linestyle, alpha=0.5)
    p = actual[:,1]
    ax[1].plot(t[:len(p)],p, linewidth=res_linewidth)
    ax[1].grid(True)


    """ YAW """
    ax[2].axhline(0, color="#AAAAAA")
    ax[2].plot(t, desired[:,2], reflinestyle)
    ax[2].plot(t, desired[:,2] -  threshold[:,2] , error_linestyle, alpha=0.5)
    ax[2].plot(t, desired[:,2] +  threshold[:,2] , error_linestyle, alpha=0.5)
    y = actual[:,2]
    ax[2].plot(t[:len(y)],y , linewidth=res_linewidth)
    ax[2].grid(True)

    plt.show()

class Policy(object):
    def action(self, state, sim_time=0, desired=np.zeros(3), actual=np.zeros(3) ):
        pass
    def reset(self):
        pass

class StaticPolicy(Policy):
    def __init__(self, motor_value):
        self.motor_value =  motor_value

    def action(self, state, sim_time=0, desired=np.zeros(3), actual=np.zeros(3)):
        return self.motor_value 

    def reset(self):
        pass

def eval(env, pi):
    """ Evaluate an environment with the given policy """
    actuals = []
    desireds = []
    pi.reset()
    ob = env.reset()
    while True:
        desired = env.omega_target
        actual = env.omega_actual
        ac = pi.action(ob, env.sim_time, desired, actual)
        ob, reward, done, info = env.step(ac)
        actuals.append(actual)
        desireds.append(desired)
        if done:
            break
    env.close()
    return desireds, actuals


def main(env_id, seed):
    env = gym.make(env_id)
    rank = MPI.COMM_WORLD.Get_rank()
    workerseed = seed + 1000000 * rank
    env.seed(workerseed)
    pi = StaticPolicy(np.ones(4))
    desireds, actuals = eval(env, pi)
    title = "PID Step Response in Environment {}".format(env_id)
    plot_step_response(np.array(desireds), np.array(actuals), title=title)

if __name__ == "__main__":

    parser = argparse.ArgumentParser("Evaluate a PID controller")
    parser.add_argument('config', help='')

    parser.add_argument('--env-id', help="The Gym environement ID", type=str,
                        default="AttFC_GyroErr-MotorVel_M4_Ep-v0")
    parser.add_argument('--seed', help='RNG seed', type=int, default=17)

    args = parser.parse_args()
    config_path = args.config
    print ("Loading config from ", config_path)
    os.environ["GYMFC_CONFIG"] = config_path

    main(args.env_id, args.seed)
