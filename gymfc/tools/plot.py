"""Helper functions for plotting""" 

import matplotlib.pyplot as plt
import math
import logging
import numpy as np

def plot_rates(ax, t, pqr_desired, pqr_actual):
    """Plot angular rates where each axis is placed in its own Matplotlib
    subplot.

    Args:
        ax: Matplotlib axis the plot will be applied to
        t: Numpy array of times
        pqr_desired: 2D numpy array of desired angular rates for each roll, 
            pitch and yaw axis.
        pqr_actual: 2D numpy array of the actual angular rates for each roll, 
            pitch and yaw axis.
    """


    unit_rotation = " (deg/s)"
    axis_label = ["p", "q", "r"]
    for i in range(len(ax)):
        ax[i].plot(t, pqr_desired[:,i], '--', c='k', label="Desired")
        ax[i].plot(t, pqr_actual[:,i], '-', c='b', label="Actual")
        #ax[i].axhline(0, color="#AAAAAA")
        ax[i].legend(loc="right")
        ax[i].grid(True)
        ax[i].set_ylabel(axis_label[i] + unit_rotation)


def plot_motor_rpms(ax, t, rpms):
    """ Plot the motor rpms values

    Args:

        ax: Matplotlib axis the plot will be applied to
        t: Numpy array of motor time 
        rpms: 2D Numpy array where the length is equal to len(t) and the width
        corresponds to the number of actuators. 
    """
    motorcolor = [['b','r','#ff8000','#00ff00'], ['m']*4]
    ax.set_ylabel("RPM")

    for i in range(len(rpms)):
        alpha = 1
        if len(self.alphas) > 0:
            alpha = self.alphas[i]
        m0 = rpms[i][:,0]
        m1 = rpms[i][:,1]
        m2 = rpms[i][:,2]
        m3 = rpms[i][:,3]
         
        ax.plot(t, m0, label="{} M1".format(self.labels[i]), linestyle=':', color=colors[i], alpha=alpha)#, color=motorcolor[i][0])
        ax.plot(t, m1, label="{} M2".format(self.labels[i]), linestyle="-", color=colors[i], alpha=alpha)#, color=motorcolor[i][1],)
        ax.plot(t, m2, label="{} M3".format(self.labels[i]), linestyle="-.", color=colors[i], alpha=alpha)#, color=motorcolor[i][2],)
        ax.plot(t, m3, label="{} M4".format(self.labels[i]), linestyle='--', color=colors[i], alpha=alpha)#color=motorcolor[i][3],
        ax.legend( loc='upper right', ncol=4)


def plot_u(ax):
    """ Plot motor control signals """
    ax.set_ylabel("Motor Control Signal (\%)")


    for i in range(len(motors)):
        alpha = 1
        if len(self.alphas) > 0:
            alpha = self.alphas[i]
        m0 = motors[i][:,0]/1000.0
        m1 = motors[i][:,1]/1000.0
        m2 = motors[i][:,2]/1000.0
        m3 = motors[i][:,3]/1000.0
         
        ax.plot(t, m0, label="{} M1".format(self.labels[i]), linestyle=':', color=colors[i], alpha=alpha)#, color=motorcolor[i][0])
        ax.plot(t, m1, label="{} M2".format(self.labels[i]), linestyle="-", color=colors[i], alpha=alpha)#, color=motorcolor[i][1],)
        ax.plot(t, m2, label="{} M3".format(self.labels[i]), linestyle="-.", color=colors[i], alpha=alpha)#, color=motorcolor[i][2],)
        ax.plot(t, m3, label="{} M4".format(self.labels[i]), linestyle='--', color=colors[i], alpha=alpha)#color=motorcolor[i][3],
        ax.legend( loc='upper right', ncol=4)

