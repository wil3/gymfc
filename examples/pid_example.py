"""An example using GymFC to test the step response of a PID controller"""
import argparse
import matplotlib.pyplot as plt
import gym
from gymfc_nf.policies import PidPolicy
from gymfc_nf.envs import *

from gymfc.tools.plot import plot_rates

if __name__ == "__main__":

    parser = argparse.ArgumentParser("Evaluate each algorithm")
    parser.add_argument('--twin', default="./gymfc_nf/twins/nf1/model.sdf",
                        help="File path of the NF1 twin/model SDF.")
    parser.add_argument('--gym-id', default="gymfc_nf-step-v1")
    parser.add_argument('--seed', help='RNG seed', type=int, default=0)
    args = parser.parse_args()

    env = gym.make(args.gym_id)
    # Need to set the aircraft model after we create the environment because 
    # the model is unique and can't be hardcoded in the gym init.
    env.set_aircraft_model(args.twin)
    env.seed(args.seed)
    env.verbose = False

    # Gains for the rate controller determined using the Ziegler-Nichols 
    # method.
    r_pid = [2.4, 33.24, 0.033]
    p_pid = [4.2, 64.33, 0.059]
    y_pid = [2, 5, 0.0]

    # Geometric measurements from the Floss 2 frame used by NF1 
    mixer=[ 
        [ 1.0, -1.0,  0.598, -1.0 ],   # REAR_R
        [ 1.0, -0.927, -0.598,  1.0 ], # FRONT_R
        [ 1.0,  1.0,  0.598,  1.0 ],   # REAR_L
        [ 1.0,  0.927, -0.598, -1.0 ], # FRONT_L
    ]

    pi = PidPolicy(r_pid, p_pid, y_pid, mixer)
    pi.reset()
    ob = env.reset()
    sim_time = 0
    states = {
        "t": [],
        "rates_actual": [],
        "rates_desired": []
    }

    while True:
        ac = pi.action(ob, env.sim_time, env.angular_rate_sp,
                       env.imu_angular_velocity_rpy)
        ob, reward, done,  _ = env.step(ac)
        if done:
            break
        states["t"].append(env.sim_time)
        states["rates_actual"].append(env.imu_angular_velocity_rpy)
        states["rates_desired"].append(env.angular_rate_sp)


    # Plot the response
    f, ax = plt.subplots(3, sharex=True, sharey=False)
    plt.suptitle("Step Response - PID Controller")
    plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
    plot_rates(ax, np.array(states["t"]), np.array(states["rates_desired"]),
               np.array(states["rates_actual"]))
    ax[-1].set_xlabel("Time (s)")
    plt.show()

