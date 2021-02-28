#!/usr/bin/env python
# Written by Aksel Kristoffersen

import numpy as np

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

