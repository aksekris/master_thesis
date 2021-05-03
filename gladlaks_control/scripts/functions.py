#!/usr/bin/env python
# Written by Aksel Kristoffersen

import numpy as np
from math import sqrt

class MassDamperSpring(object):
    """A mass-damper-spring object"""
    def __init__(self, m, d, k):
        self.m = m
        self.d = d
        self.k = k
        self.K = 1/d
        self.T = m/d

    def pid_pole_placement_algorithm(self, omega_b, zeta):
        """PID pole-placement based on algorithm described in Fossen's 
        Handbook of marine craft hydrodynamics and motion control, Chapter 15.3.

        Args:
            m           The mass coefficient
            d           The damper coefficient
            k           The spring coefficient
            omega_b     The control bandwidth
            zeta        The relative damping ratio
        Returns:
            K_p         The proportional gain
            K_d         The derivative gain
            K_i         The integral gain
        """
        self.zeta = zeta
        self.omega_b = omega_b
        self.omega_n = omega_b/sqrt(1 - 2*zeta**2+sqrt(4*zeta**4 - 4*zeta**2 + 2))
        K_p = self.m*self.omega_n**2 - self.k
        K_d = 2*zeta*self.omega_n*self.m - self.d
        K_i = self.omega_n*K_p/10

        return K_p, K_d, K_i

    def pi_pole_placement_algorithm(self, omega_b, zeta):
        self.zeta = zeta
        self.omega_b = omega_b
        self.omega_n = omega_b/sqrt(1 - 2*zeta**2+sqrt(4*zeta**4 - 4*zeta**2 + 2))
        K_p = (2*zeta*self.omega_n*self.T - 1)/self.K
        K_i = self.omega_n**2*self.T/((2*zeta*self.omega_n*self.T - 1))
        return K_p, K_i

class FirstOrderSystem(object):

    def __init__(self, m, d):
        self.m = m
        self.d = d
        self.k = 1/d
        self.t = m/d
    def pi_pole_placement_algorithm(self, omega_b, zeta):
        self.zeta = zeta
        self.omega_b = omega_b
        self.omega_n = omega_b/sqrt(1 - 2*zeta**2+sqrt(4*zeta**4 - 4*zeta**2 + 2))
        K_p = (2*zeta*self.omega_n*self.t - 1)/self.k
        K_i = self.omega_n**2*self.t/((2*zeta*self.omega_n*self.t - 1))
        return K_p, K_i

class PIDController(object):
	"""1D PID controller using the state derivative, Euler integration, anit-windup
	and with the possibility of adding a feed-forward input"""
	def __init__(self, K_p, K_d, K_i, u_max):
		"""Initialize the PID controller

		Args:
			K_p	(float)  	Proportional gain
			K_i	(float)  	Integral gain
			K_d	(float)  	Derivative gain
			u_max (float)	Upper and lower output limit
			t (float)		Current time when initializing the controller
		"""
		self.K_p = K_p
		self.K_i = K_i
		self.K_d = K_d
		self.u_max = u_max

	def initialize(self, t):
		self.prev_t = t
		self.integral = 0

	def regulate(self, x_err, x_dt, t, u_ff=0):
		"""Calculate the controller output

		Args:
			x_err (float)	The state error
			x_dt (float)	The state derivative
			t (float)	  	The current time
			u_ff (float)	Optional feed-forward input

		Returns:
			u (float):		The controller output

		"""

		dt = t - self.prev_t
		if self.prev_t > 0.0 and dt > 0.0:
			self.integral += x_err*dt

		u_unsat = u_ff - (self.K_p*x_err + self.K_d*x_dt + self.K_i*self.integral)
		self.prev_t = t

		if abs(u_unsat) > self.u_max:
			# Anti-wind-up
			u = np.sign(u_unsat) * self.u_max
			self.integral += - (dt/self.K_i) * (u - u_unsat)
		else:
			u = u_unsat

		return u

class MassDamperSpringSystem(object):
    """
    A multi dimensional mass damper spring system with option
    to use velocity and acceleration saturation limits
    """
    def __init__(self, delta, omega, **kwargs):
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
        n = len(omega)
        self.A_d = np.block([[np.zeros((n, n)), np.eye(n)], [-omega_bf**2, -2*delta_bf*omega_bf]])
        self.B_d = np.block([[np.zeros((n, n))], [omega_bf**2]])
        self.x_bf = np.empty(n*2)
        self.prev_t = 0
        
    def initialize(self, x, x_dot, t):
        self.x_bf = np.concatenate((x, x_dot), axis=0)
        self.prev_t = t

    def simulate(self, r, t):
        """Function for simulating one timestep of the system using Euler integration

        Args:
            r (list):       Input signal
        
        Returns:
            x (list):       The states
            x_dot (list):   The state derivatives

        """

        # Simulate system
        x_bf_dot = np.dot(self.A_d, self.x_bf) + np.dot(self.B_d, r)


        if hasattr(self, 'x_dot_max'):
            # Saturation for velocities
            bool_list = np.abs(x_bf_dot[0:(len(x_bf_dot)/2)]) > self.x_dot_max
            for index in [i for i, x in enumerate(bool_list) if x]:
                x_bf_dot[index] = np.sign(x_bf_dot[index]) *self.x_dot_max[index]
        
        if hasattr(self, 'x_ddot_max'):
            # Saturation for accelerations
            bool_list = np.abs(x_bf_dot[(len(x_bf_dot)/2):]) > self.x_ddot_max
            for index in [i for i, x in enumerate(bool_list) if x]:
                x_bf_dot[(len(x_bf_dot)/2)+index] = np.sign(x_bf_dot[(len(x_bf_dot)/2)+index]) *self.x_ddot_max[index]
            
        
        # Euler integration
        dt = t - self.prev_t
        self.x_bf = self.x_bf + dt*x_bf_dot
        self.prev_t = t

        if hasattr(self, 'x_dot_max'):
            # Another saturation for velocities
            bool_list = np.abs(self.x_bf[(len(self.x_bf)/2):]) > self.x_dot_max
            for index in [i for i, x in enumerate(bool_list) if x]:
                self.x_bf[(len(self.x_bf)/2)+index] = np.sign(self.x_bf[(len(self.x_bf)/2)+index]) *self.x_dot_max[index]

        _ , x_ddot_arr = np.array_split(x_bf_dot, 2)
        x_arr, x_dot_arr = np.array_split(self.x_bf, 2)
        x = x_arr.tolist()
        x_dot = x_dot_arr.tolist()
        x_ddot = x_ddot_arr.tolist()
    
        return x, x_dot, x_ddot

class LowPassFilter(object):
    """A multivarible low-pass filter"""
    def __init__(self, x, omega, t):
        """ Initialize the low-pass filter

        Args:
            x (list):           The inital states
            omega (list):       The natural frequencies
            t (float):          The current time

        """
        n = len(omega)
        self.x = x
        self.omega_bf = np.diag(omega)
        self.prev_t = t

    def simulate(self, r, t):
        """Function for simulating one timestep of the low-pass filter using euler integration

        Args:
            r (list):       Input signal
        
        Returns:
            x (list):       Output signal

        """

        x_dot = np.dot(self.omega_bf, (np.array(r) - np.array(self.x)))
        dt = t - self.prev_t
        self.x = self.x + dt*x_dot
        self.prev_t = t
        x = self.x.tolist()

        return x

def pid_pole_placement_algorithm(m, d, k, omega_b, zeta):

    """PID pole-placement based on algorithm described in Fossen's 
    Handbook of marine craft hydrodynamics and motion control, Chapter 15.3.

    Args:
        m           The mass coefficient
        d           The damper coefficient
        k           The spring coefficient
        omega_b     The control bandwidth
        zeta        The relative damping ratio
    Returns:
        K_p         The proportional gain
        K_d         The derivative gain
        K_i         The integral gain
    """

    omega_n = omega_b/sqrt(1-2*zeta**2+sqrt(4*zeta**4-4*zeta**2+2))
    K_p = m*omega_n**2-k
    K_d = 2*zeta*omega_n*m-d
    K_i = omega_n*K_p/10

    return K_p, K_d, K_i

def quaternion_to_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Args:
        Q (list) A 4 element array representing the quaternion in ROS 


    Return:
        rot_matrix (np.arryay) A 3x3 element matrix representing the full 3D rotation matrix. 
        This rotation matrix converts a point in the local reference 
        frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[3]
    q1 = Q[0]
    q2 = Q[1]
    q3 = Q[2]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

class HeadingAutopilot(PIDController):
    """Heading autopilot using a PID controller"""

    def __init__(self, K_p, K_d, K_i, tau_sat):
        super(HeadingAutopilot, self).__init__(K_p, K_d, K_i, tau_sat)

    def calculate_control_torque(self, yaw_err, yaw_dt, t, yaw_ff=0):
        """Calculate the control torque for heading control using a PID controller

        Args:
            yaw_err     The heading angle error
            yaw_dt      The yaw derivative
            t           The time now
            yaw_ff      Optional feed-forward output

        Returns:
            float:      The control torque in yaw, tau_6

        """
        if yaw_err < -np.pi or yaw_err >= np.pi:
            # Map angle to [-pi, pi)
            yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi

        tau_6 = self.regulate(yaw_err, yaw_dt, t, yaw_ff)

        return tau_6
