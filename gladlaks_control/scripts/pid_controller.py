#!/usr/bin/env python
# Written by Aksel Kristoffersen

import numpy as np

class PIDController(object):

	def __init__(self, K_p, K_d, K_i, u_sat, t):
		"""Initialize the PID controller

		Args:
			K_p	  	Proportional gain
			K_i	  	Integral gain
			K_d	  	Derivative gain
			u_sat	Output saturation limit
			t		Current time when initializing the controller
		"""
		self.K_p = K_p
		self.K_i = K_i
		self.K_d = K_d
		self.u_sat = u_sat
		self.integral = 0
		self.prev_x_err = 0
		self.prev_t = t

	def regulate(self, x_err, x_dt, t, u_ff=0):
		"""Calculate the controller output

		Args:
			x_err	  	The state error
			x_dt		The state derivative
			t	  		The current time
			u_ff		Optional feed-forward output

		Returns:
			float:		The controller output u

		"""
		dt = t - self.prev_t
		if self.prev_t > 0.0 and dt > 0.0:
			self.integral += x_err*dt

		u_unsat = u_ff - (self.K_p*x_err + self.K_d*x_dt + self.K_i*self.integral)
		self.prev_x_err = x_err
		self.prev_t = t

		if abs(u_unsat) > self.u_sat:
			# Anti-wind-up
			u = np.sign(u_unsat) * self.u_sat
			self.integral += - (dt/self.K_i) * (u - u_unsat)
		else:
			u = u_unsat

		return u

