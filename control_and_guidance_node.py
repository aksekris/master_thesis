#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
import numpy as np
from control_system import DPControlSystem
from auv_simulator import AUVSimulator
from path import Path

class GuidanceAndControlNode:
    def __init__(self):
        
        # Initialize the ROS-node
        rospy.init_node('control_system')
        while rospy.get_time() == 0:
            continue
        rospy.Subscriber('/eskf_localization/pose', Odometry, self.navigation_callback)

        self.frequency = 

        self.auv_model = AUVModel(m, r_g, r_b, inertia, volume, M_A, D, rho=997, g=9.81, dot_eta_c=[0,0,0,0,0,0])
        self.actuator_model = ActuatorModel(input_matrix, rotor_time_constant, u_max, u_min)
        self.dp_control_system = DPControlSystem(M, D, omega_b, zeta)
        self.reference_model = AUVSimulator()
        self.reference_model.set_initial_conditions(eta, nu, dot_nu, tau, rospy.get_time())
        self.path = Path()
        self.mode == 'path_following'

        # Add whatever mode
        if self.mode == 'path_following':
            waypoints = [[0, 0, 0], [1, 0, 1], [0, 1, 0], [0, 0, 0]]
            self.path.generate_G0_path(waypoints)
            self.path_following_controller = VirtualTarget(path, auv_model, actuator_model, dp_control_system)
        elif self.mode == 'pose_hold':
            self.eta_r = [0, 0, 0, 0, 0, 0]
        self.get_pose = False

    def navigation_callback(self, msg):
        if self.get_pose:
            self.eta = extract_from_pose(msg.pose.pose)
            self.nu = extract_from_twist(msg.twist.twist)
            self.get_pose = False
        else:
            pass

    def get_state_estimates(self):
        self.get_pose = True
        while self.get_pose:
            continue

    def publish_control_forces(self):
        while not rospy.is_shutdown():
            try:
                self.get_state_estimates()
                if self.mode == 'path_following':
                    eta_r, nu_r, dot_nu_r = self.path_following_controller.generate_reference_trajectories(self.eta, self.nu, rospy.get_time())
                if self.mode == 'pose_hold'
                    eta_r = self.eta_r
                    nu_r = [0, 0, 0, 0, 0, 0]
                    dot_nu_r = [0, 0, 0, 0, 0, 0]
                eta_d, nu_d, dot_nu_d = reference_model.generate_trajectory_for_dp(rospy.get_time(), 1, 1/self.frequency, eta_ref, nu_ref=nu_r, dot_nu_ref=dot_nu_r)
                tau_c = self.dp_control_system.pid_regulate(self.eta, self.nu, eta_d, nu_d, dot_nu_d, rospy.get_time())

                msg = WrenchStamped()
                msg.header.stamp = rospy.get_rostime()
                msg.header.frame_id = "gladlaks/base_link_ned"
                msg.wrench.force.x = tau_c[0]
                msg.wrench.force.y = tau_c[1]
                msg.wrench.force.z = tau_c[2]
                msg.wrench.torque.x = tau_c[3]
                msg.wrench.torque.y = tau_c[4]
                msg.wrench.torque.z = tau_c[5]
                self.rate.sleep()
                self.pub.publish(msg)
            except rospy.ROSInterruptException:
                pass

def extract_from_pose(pose):
    quaternions = pose.orientation
    euler_angles = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])
    position = (pose.position.x, pose.position.y, pose.position.z)
    return [position[0], position[1], position[2], euler_angles[0], euler_angles[1], euler_angles[2]]

def extract_from_twist(twist):
    linear = (twist.linear.x, twist.linear.y, twist.linear.z)
    angular = (twist.angular.x, twist.angular.y, twist.angular.z)
    return [linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]]


if __name__ == '__main__':
    try:
        node = GuidanceAndControlNode()
        node.publish_control_forces()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

