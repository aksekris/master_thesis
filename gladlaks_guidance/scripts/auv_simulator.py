#!/usr/bin/env python
# Written by Aksel Kristoffersen

import numpy as np
from quadratic_programming import quadprog_solve_qp
from nav_msgs.msg import Path

class AUVSimulator():
    def __init__(self):
        rospy.init_node('auv_simulator')
        while rospy.get_time() == 0:
            continue
        self.pub = rospy.Publisher('/gladlaks/guidance_system/auv_simulator/path', Path, queue_size=1)
        M_RB = rospy.get_param("/auv_dynamics/M_RB")
        M_A = rospy.get_param("/auv_dynamics/M_A")
        self.M = M_RB + M_A
        self.D = rospy.get_param("/auv_dynamics/D")
        duty_cycle_limits = rospy.get_param("/thrusters/duty_cycle_limits")
        propeller_revolution_limits = [(x - 1500)/0.1 for x in duty_cycle_limits]
        rotor_constant = rospy.get_param("/thrusters/rotor_constant")
        control_input_limits = [rotor_constant*abs(x)*x for x in propeller_revolution_limits]
        self.u_min = control_input_limits[0]
        self.u_max = control_input_limits[1]

    def simulate(self, eta, nu, ):
        self.eta = eta
        self.nu = nu


    def publish(self):
        msg = Path()



