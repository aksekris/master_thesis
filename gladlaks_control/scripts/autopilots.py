#!/usr/bin/env python
# Written by Aksel Kristoffersen

import numpy as np
from pid_controller import PIDController

class HeadingAutopilot(PIDController):
    """Heading autopilot using a PID controller"""

    def __init__(self, K_p, K_d, K_i, tau_sat, t):
        super(HeadingAutopilot, self).__init__(K_p, K_d, K_i, tau_sat, t)

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


