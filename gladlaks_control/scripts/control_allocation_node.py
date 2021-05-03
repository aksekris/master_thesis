#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from vortex_msgs import Pwm

class ThrusterAllocation:

    def __init__(self):
        
        rospy.init_node('control_allocation')
        sub = rospy.Subscriber('/gladlaks/thruster_manager/input_stamped', WrenchStamped, self.callback())
        self.pub = rospy.Publisher('/pwm', Pwm, queue_size=1)
        self.thruster_map = rospy.get_param("/thrusters/map")
        self.duty_cycle_offset = rospy.get_param("/thrusters/duty_cycle_off_set")
        self.rotor_constant = rospy.get_param("/thrusters/rotor_constant")
        input_matrix = rospy.get_param("/thrusters/input_matrix")
        input_matrix_arr = np.array(input_matrix)
        self.pseudo_inv_matrix = np.dot(input_matrix_arr.T, np.linalg.inv(np.dot(input_matrix_arr, input_matrix_arr.T)))

    def callback(self, msg):
        tau_c = [0, 0, 0, 0, 0, 0]
        tau_c[0] = mgs.wrench.force.x
        tau_c[1] = mgs.wrench.force.y
        tau_c[2] = mgs.wrench.force.z
        tau_c[3] = mgs.wrench.torque.x
        tau_c[4] = mgs.wrench.torque.y
        tau_c[5] = mgs.wrench.torque.z

        thrust = np.dot(self.pseudo_inv_matrix, tau_c)
        n = np.sign(thrust)*np.sqrt(np.abs(thrust)/ self.rotor_constant)
        duty_cycle = 0.1*n + 1500
        duty_cycle = [sum(x) for x in zip(duty_cycle, self.duty_cycle_offset)]
        pwm_msg = Pwm()
        pwm_msg.pins = self.thruster_map
        pwm_msg.positive_width_us = duty_cycle

        self.pub.publish(pwm_msg)
        
    



if __name__ == '__main__':
    try:    
        node = ThrusterAllocation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


