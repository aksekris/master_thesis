#!/usr/bin/env python
# Written by Aksel Kristoffersen

import numpy as np
import math 
import matplotlib.pyplot as plt

class MassDamperSpringSystem(object):
    """
    A multi dimensional mass damper spring system with option
    to use velocity and acceleration saturation limits
    """
    def __init__(self, x, x_dot, delta, omega, t, **kwargs):
        """ Initialize the mass damper spring system as the one described in Fossen's Handbook 
        of marine craft hydrodynamics and motion control, Chapter 12.1.1

        Args:
            x (list):           The inital states
            x_dot (list):       The initial state derivatives
            delta (list):       The relative damping ratios
            omega (list):       The natural frequencies
            t (float):

        Other Parameters:
            x_dot_max (list):   The velocity limit
            x_ddot_max (list):  The acceleration limt

        """
        if 'x_dot_max' in kwargs:
            self.x_dot_max = kwargs['x_dot_max']

        if 'x_ddot_max' in kwargs:
            self.x_ddot_max = kwargs['x_ddot_max']


        delta_bf = np.diag(delta)
        omega_bf = np.diag(omega)
        n = len(delta)
        self.A_d = np.block([[np.zeros((n, n)), np.eye(n)], [-omega_bf**2, -2*delta_bf*omega_bf]])
        self.B_d = np.block([[np.zeros((n, n))], [omega_bf**2]])
        self.prev_t = t
        self.x_bf = np.concatenate((x, x_dot), axis=0)

    def simulate(self, r, t):
        """Function for simulating one timestep of the system

        Args:
            r (list):       Input signal
        
        Returns:
            x (list):       Output signal
            x_dot (list):   Output signal derivative

        """
        x_bf_dot = np.dot(self.A_d, self.x_bf) + np.dot(self.B_d, r)


        if hasattr(self, 'x_dot_max'):
            # Saturation limits for velocities
            bool_list = np.abs(x_bf_dot[0:(len(x_bf_dot)/2)]) > self.x_dot_max
            for index in [i for i, x in enumerate(bool_list) if x]:
                x_bf_dot[index] = np.sign(x_bf_dot[index]) *self.x_dot_max[index]
        
        if hasattr(self, 'x_ddot_max'):
            # Saturation limits for accelerations
            bool_list = np.abs(x_bf_dot[(len(x_bf_dot)/2):]) > self.x_ddot_max
            for index in [i for i, x in enumerate(bool_list) if x]:
                x_bf_dot[(len(x_bf_dot)/2)+index] = np.sign(x_bf_dot[(len(x_bf_dot)/2)+index]) *self.x_ddot_max[index]
            

        dt = t - self.prev_t
        self.x_bf = self.x_bf + dt*x_bf_dot

        if hasattr(self, 'x_dot_max'):
            # Another saturation limits for velocities
            bool_list = np.abs(self.x_bf[(len(self.x_bf)/2):]) > self.x_dot_max
            for index in [i for i, x in enumerate(bool_list) if x]:
                self.x_bf[(len(self.x_bf)/2)+index] = np.sign(self.x_bf[(len(self.x_bf)/2)+index]) *self.x_dot_max[index]

        x_arr, x_dot_arr = np.array_split(self.x_bf, 2)
        x = x_arr.tolist()
        x_dot = x_dot_arr.tolist()
    
        return x, x_dot

class LowPassFilter(object):
    """A multi dimensional low-pass filter"""
    def __init__(self, x, omega, t):
        """ Initialize the low-pass filter

        Args:
            x (list):           The inital states
            omega (list):       The relative damping ratios
            t (float):

        Other Parameters:
            x_dot_max (list):   The velocity limit
            x_ddot_max (list):  The acceleration limt

        """




if __name__ == '__main__':
    x = [0, 0, 0]
    x_dot = [0, 0, 0]
    delta = [1, 1, 0.8] 
    omega = [0.5, 0.5, 0.5]
    system = MassDamperSpringSystem(x, x_dot, delta, omega, 0)

    plot1 = []
    plot2 = []
    plot3 = []

    for t in np.linspace(0, 2.0, num=100):
        x, x_dot = system.simulate([1, 1, 1], t)
        print('x:')
        print(x)
        print('x_dt:')
        print(x_dot)
        plot1.append(x[0])
        plot2.append(x[1])
        plot3.append(x[2])

    plt.plot(plot3)
    plt.ylabel('Position')
    plt.show()
    plt.close('all')