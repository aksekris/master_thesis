#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from tf.transformations import euler_from_quaternion
from pid_controller import PIDController


class HeadingAutopilot(PIDController):
    """Heading autopilot using a PID controller"""

    def __init__(self, K_p, K_d, K_i, tau_sat):
        super(HeadingAutopilot, self).__init__(K_p, K_d, K_i, tau_sat)

    def calculate_control_torque(self, yaw_err, yaw_dt, t):
        """Calculate the control torque for heading control using a PID controller

        Args:
            yaw_err     The heading angle error
            yaw_dt      The yaw derivative
            t           The time now

        Returns:
            float:      The control torque tau

        """
        if yaw_err < -np.pi or yaw_err >= np.pi:
            # Map angle to [-pi, pi)
            yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi

        return self.regulate(yaw_err, yaw_dt, t)



