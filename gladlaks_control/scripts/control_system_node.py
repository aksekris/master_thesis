#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
import threading
import math
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from autopilots import HeadingAutopilot
from pid_controller import PIDController
from pid_pole_placement_algorithm import pid_pole_placement_algorithm


class ControlSystem:
    def __init__(self):

        rospy.init_node('control_system')
        while rospy.get_time() == 0:
            continue
        pose_sub = rospy.Subscriber('/eskf_localization/pose', Odometry, self.pose_callback)
        input_sub = rospy.Subscriber('/gladlaks/control_system/input_pose', Odometry, self.input_callback)
        self.pub = rospy.Publisher('/gladlaks/thruster_manager/input_stamped', WrenchStamped, queue_size=1)
        self.controller_frequency = rospy.get_param("/control_system/controller_frequency")
        self.get_pose = False
        self.eta = [None, None, None, None, None, None]
        self.eta_d = [0, 0, 0.5, 0, 0, 0] # Placeholder
        self.nu = [None, None, None, None, None, None]
        self.nu_d = [None, None, None, None, None, None] # Placeholder

        M_RB = rospy.get_param("/auv_dynamics/M_RB")
        M_A = rospy.get_param("/auv_dynamics/M_A")

        # Initialize the surge controller
        m = M_RB[0][0]+M_A[0][0]
        d = -rospy.get_param("/auv_dynamics/D")[0]
        k = 0
        omega_b = rospy.get_param("/control_system/surge_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/surge_controller/relative_damping_ratio")
        tau_sat = rospy.get_param("/control_system/surge_controller/torque_saturation_limit")
        K_p, K_d, K_i = pid_pole_placement_algorithm(m, d, k, omega_b, zeta)
        self.surge_controller = PIDController(K_p, K_d, K_i, tau_sat, rospy.get_time())

        # Initialize the sway controller
        m = M_RB[1][1]+M_A[1][1]
        d = -rospy.get_param("/auv_dynamics/D")[1]
        k = 0
        omega_b = rospy.get_param("/control_system/sway_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/sway_controller/relative_damping_ratio")
        tau_sat = rospy.get_param("/control_system/sway_controller/torque_saturation_limit")
        K_p, K_d, K_i = pid_pole_placement_algorithm(m, d, k, omega_b, zeta)
        self.sway_controller = PIDController(K_p, K_d, K_i, tau_sat, rospy.get_time())

        # Initialize the heading controller
        m = M_RB[5][5]+M_A[5][5]
        d = -rospy.get_param("/auv_dynamics/D")[5]
        k = 0
        omega_b = rospy.get_param("/control_system/heading_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/heading_controller/relative_damping_ratio")
        tau_sat = rospy.get_param("/control_system/heading_controller/torque_saturation_limit")
        K_p, K_d, K_i = pid_pole_placement_algorithm(m, d, k, omega_b, zeta)
        self.heading_controller = HeadingAutopilot(K_p, K_d, K_i, tau_sat, rospy.get_time())
        
        # Initialize the depth controller
        m = M_RB[2][2]+M_A[2][2]
        d = -rospy.get_param("/auv_dynamics/D")[2]
        k = 0  
        omega_b = rospy.get_param("/control_system/depth_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/depth_controller/relative_damping_ratio")
        tau_sat = rospy.get_param("/control_system/depth_controller/torque_saturation_limit")
        K_p, K_d, K_i = pid_pole_placement_algorithm(m, d, k, omega_b, zeta)
        self.depth_controller = PIDController(K_p, K_d, K_i, tau_sat, rospy.get_time())

    def pose_callback(self, msg):
        if self.get_pose:
            self.eta = extract_eta_from_odom(msg)
            self.nu = extract_nu_from_odom(msg)
            self.get_pose = False
        else:
            pass
    
    def input_callback(self, msg):
        self.eta_d = extract_eta_from_odom(msg)
        self.nu_d = extract_nu_from_odom(msg)

    def get_state_estimates(self):
        self.get_pose = True
        while self.get_pose:
            continue

    def calculate_control_forces(self):
        tau_1_ned = self.surge_controller.regulate((self.eta[0] - self.eta_d[0]), self.nu[0], rospy.get_time())
        tau_2_ned = self.sway_controller.regulate((self.eta[1] - self.eta_d[1]), self.nu[1], rospy.get_time())
        tau_1 = math.cos(self.eta[5]) * tau_1_ned + math.sin(self.eta[5]) * tau_2_ned 
        tau_2 = - math.sin(self.eta[5]) * tau_1_ned + math.cos(self.eta[5]) * tau_2_ned
        tau_3 = self.depth_controller.regulate((self.eta[2] - self.eta_d[2]), self.nu[2], rospy.get_time(), u_ff=23)                
        tau_4 = 0
        tau_5 = 0
        tau_6 = self.heading_controller.calculate_control_torque((self.eta[5] - self.eta_d[5]), self.nu[5], rospy.get_time())
        print('errors: Surge, Sway, Yaw' )
        print(self.eta[0] - self.eta_d[0], self.eta[1] - self.eta_d[1], self.eta[5] - self.eta_d[5])
        return [tau_1, tau_2, tau_3, tau_4, tau_5, tau_6]

    def publish_control_forces(self):
        rate = rospy.Rate(self.controller_frequency)
        while not rospy.is_shutdown():
            try:
                self.get_state_estimates()
                tau = self.calculate_control_forces()
                msg = WrenchStamped()
                msg.header.stamp = rospy.get_rostime()
                msg.header.frame_id = "gladlaks/base_link_ned"
                msg.wrench.force.x = tau[0]
                msg.wrench.force.y = tau[1]
                msg.wrench.force.z = tau[2]
                msg.wrench.torque.x = tau[3]
                msg.wrench.torque.y = tau[4]
                msg.wrench.torque.z = tau[5]
                rate.sleep()
                self.pub.publish(msg)
            except rospy.ROSInterruptException:
                pass
        
def extract_eta_from_odom(msg):
    quaternions = msg.pose.pose.orientation
    euler_angles = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])
    position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    eta = [position[0], position[1], position[2], euler_angles[0], euler_angles[1], euler_angles[2]]
    return eta

def extract_nu_from_odom(msg):
    linear = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
    angular = (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z)
    nu = [linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]]
    return nu


if __name__ == '__main__':
    try:
        node = ControlSystem()
        node.publish_control_forces()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

"""
node = ControlSystem()
        worker = threading.Thread(target=node.publish_control_forces)
        worker.start()
        rospy.spin()
"""