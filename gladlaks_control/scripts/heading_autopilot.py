#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from tf.transformations import euler_from_quaternion
from pid_controller import PIDController
from pid_pole_placement_algorithm import pid_pole_placement_algorithm

class HeadingAutopilot:
    """Heading autopilot based on PID pole-placement"""

    def __init__(self, m, d, k, omega_b, zeta, tau_sat):
        """Initialize the autopilot

        Args: 
            m           The mass coefficient
            d           The damper coefficient
            k           The spring coefficient
            omega_b     The control bandwidth
            zeta        The relative damping ratio
            tau_sat     The input torque saturation limit

        """
        (K_p, K_d, K_i) = pid_pole_placement_algorithm(m, d, k, omega_b, zeta)
        self.PID = PIDController(K_p, K_d, K_i, tau_sat)

    def calculate_control_torque(self, yaw, yaw_d, yaw_dt, t):
        """Calculate the control torque for heading control using a PID controller

        Args:
            yaw         The heading angle
            yaw_d       The desired heading angle
            yaw_dt      The yaw derivative
            t           The time now

        Returns:
            float:      The control torque tau

        """
        yaw_err = yaw - yaw_d
        if yaw_err < -np.pi or yaw_err >= np.pi:
            # Map angle to [-pi, pi)
            yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi

        return self.PID.regulate(yaw_err, yaw_dt, t)
        