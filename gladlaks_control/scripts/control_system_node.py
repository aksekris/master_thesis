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
from mass_damper_spring import MassDamperSpring, FirstOrderSystem
from reference_models import LowPassFilter, MassDamperSpringSystem
from functions import quaternion_to_rotation_matrix
from gladlaks_control.msg import ReferenceTrajectoryStamped


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
        self.eta_r = [0, 0, 1, 0, 0, 0]
        self.nu_r = [0, 0, 0, 0, 0, 0]
        self.control_type = ''
        self.prev_control_type = 'unique'

        M_RB = rospy.get_param("/auv_dynamics/M_RB")
        M_A = rospy.get_param("/auv_dynamics/M_A")

        # Initialize the surge dp controller
        m = M_RB[0][0]+M_A[0][0]
        d = -rospy.get_param("/auv_dynamics/D")[0]
        k = 0
        self.surge_sub_system = MassDamperSpring(m, d, k)
        omega_b = rospy.get_param("/control_system/surge_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/surge_controller/relative_damping_ratio")
        K_p, K_d, K_i = self.surge_sub_system.pid_pole_placement_algorithm(omega_b, zeta)
        tau_sat = rospy.get_param("/control_system/surge_controller/torque_saturation_limit")
        self.surge_dp_controller = PIDController(K_p, K_d, K_i, tau_sat)

        # Initialize the sway dp controller
        m = M_RB[1][1]+M_A[1][1]
        d = -rospy.get_param("/auv_dynamics/D")[1]
        k = 0
        self.sway_sub_system = MassDamperSpring(m, d, k)
        omega_b = rospy.get_param("/control_system/sway_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/sway_controller/relative_damping_ratio")
        K_p, K_d, K_i = self.sway_sub_system.pid_pole_placement_algorithm(omega_b, zeta)
        tau_sat = rospy.get_param("/control_system/sway_controller/torque_saturation_limit")
        self.sway_dp_controller = PIDController(K_p, K_d, K_i, tau_sat)

        # Initialize the depth controller
        m = M_RB[2][2]+M_A[2][2]
        d = -rospy.get_param("/auv_dynamics/D")[2]
        k = 0 
        self.heave_sub_system = MassDamperSpring(m, d, k)
        omega_b = rospy.get_param("/control_system/depth_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/depth_controller/relative_damping_ratio")
        tau_sat = rospy.get_param("/control_system/depth_controller/torque_saturation_limit")
        K_p, K_d, K_i = self.heave_sub_system.pid_pole_placement_algorithm(omega_b, zeta)
        self.depth_controller = PIDController(K_p, K_d, K_i, tau_sat)

        # Initialize the heading controller
        m = M_RB[5][5]+M_A[5][5]
        d = -rospy.get_param("/auv_dynamics/D")[5]
        k = 0
        self.yaw_sub_system = MassDamperSpring(m, d, k)
        omega_b = rospy.get_param("/control_system/heading_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/heading_controller/relative_damping_ratio")
        K_p, K_d, K_i = self.yaw_sub_system.pid_pole_placement_algorithm(omega_b, zeta)
        tau_sat = rospy.get_param("/control_system/heading_controller/torque_saturation_limit")
        self.heading_controller = HeadingAutopilot(K_p, K_d, K_i, tau_sat)

        # Initialize the surge speed controller
        omega_b = rospy.get_param("/control_system/surge_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/surge_controller/relative_damping_ratio")
        zeta = 0.85 # !!!!!!!!!!!!!!!!!!!!!!!efqawrgqawe rgqergwewaer gwerg wqer gbewr b
        K_p, K_i = self.surge_sub_system.pi_pole_placement_algorithm(omega_b, zeta)
        print('Surge speed controller gains:')
        print((K_p, K_i))
        tau_sat = rospy.get_param("/control_system/surge_controller/torque_saturation_limit")
        self.surge_speed_controller = PIDController(K_p, 0, K_i, tau_sat)

        # Initialize the sway speed controller
        omega_b = rospy.get_param("/control_system/sway_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/sway_controller/relative_damping_ratio")
        zeta = 0.85 # !!!!!!!!!!!!!!!!!!!!!!!efqawrgqawe rgqergwewaer gwerg wqer gbewr b
        K_p, K_i = self.sway_sub_system.pi_pole_placement_algorithm(omega_b, zeta)
        print('Sway speed controller gains:')
        print((K_p, K_i))
        tau_sat = rospy.get_param("/control_system/sway_controller/torque_saturation_limit")
        self.sway_speed_controller = PIDController(K_p, 0, K_i, tau_sat)

        # Initialize the reference model
        eta = rospy.get_param("/initial_conditions/auv/eta")
        nu = [0, 0, 1, 0, 0, 0]
        delta = [1, 1, 1, 1, 1, 1]
        omega = [self.surge_sub_system.omega_b*0.5, 
                self.sway_sub_system.omega_b*0.5, 
                self.heave_sub_system.omega_b*0.5,
                1,
                1, 
                self.yaw_sub_system.omega_b*0.5]
        # vel_limits = 
        # acc_limits = 
        print('Reference model natural frequency')
        print(self.surge_sub_system.omega_b*0.5)
        self.low_pass_filter = LowPassFilter(eta, omega, rospy.get_time())
        self.mass_damper_spring_system = MassDamperSpringSystem(delta, omega)
        
    def pose_callback(self, msg):
        if self.get_pose:
            self.eta = extract_eta_from_odom(msg)
            self.nu = extract_nu_from_odom(msg)
            self.get_pose = False
        else:
            pass
    
    def input_callback(self, msg):
        self.eta_r = extract_eta_from_odom(msg)
        self.nu_r = extract_nu_from_odom(msg)
        self.control_type = msg.child_frame_id

    def get_state_estimates(self):
        self.get_pose = True
        while self.get_pose:
            continue

    def generate_trajectory(self):
        if self.control_type == 'dp_control':
            if not self.control_type == self.prev_control_type:
                x = self.eta
                self.mass_damper_spring_system.initialize(x, self.nu, rospy.get_time())
                print('Enter dp mode')
        r = self.low_pass_filter.simulate(self.eta_r, rospy.get_time())
        if self.control_type == 'course_control':
            if not self.control_type == self.prev_control_type:
                x = self.eta
                x[0:2] = self.nu[0:2]
                self.mass_damper_spring_system.initialize(x, self.nu, rospy.get_time())
                print('Enter course mode')
            r[0:2] = self.nu_r[0:2]
        eta_d, eta_dot_d, eta_ddot_d = self.mass_damper_spring_system.simulate(r, rospy.get_time())
        return eta_d, eta_dot_d, eta_ddot_d

    def calculate_control_forces(self, eta_d, eta_dot_d, eta_ddot_d):
        self.depth_controller.initialize(rospy.get_time())
        self.heading_controller.initialize(rospy.get_time())
        tau_3_ref_ff = self.heave_sub_system.d * eta_dot_d[2] + self.heave_sub_system.m * eta_ddot_d[2]
        tau_3 = tau_3_ref_ff + self.depth_controller.regulate((self.eta[2] - eta_d[2]), self.nu[2], rospy.get_time(), u_ff=23)                
        tau_4 = 0
        tau_5 = 0
        tau_6_ref_ff = self.yaw_sub_system.d * eta_dot_d[5] + self.yaw_sub_system.m * eta_ddot_d[5]
        tau_6 = tau_6_ref_ff + self.heading_controller.calculate_control_torque((self.eta[5] - eta_d[5]), self.nu[5], rospy.get_time())
        if self.control_type == 'dp_control':
            if not self.control_type == self.prev_control_type:
                self.surge_dp_controller.initialize(rospy.get_time())
                self.sway_dp_controller.initialize(rospy.get_time())
            tau_1_ned_ref_ff = self.surge_sub_system.d * eta_dot_d[0] + self.surge_sub_system.m * eta_ddot_d[0]
            tau_1_ned = tau_1_ned_ref_ff + self.surge_dp_controller.regulate((self.eta[0] - eta_d[0]), self.nu[0], rospy.get_time())
            tau_2_ned_ref_ff = self.sway_sub_system.d * eta_dot_d[1] + self.sway_sub_system.m * eta_ddot_d[1]
            tau_2_ned = tau_2_ned_ref_ff + self.sway_dp_controller.regulate((self.eta[1] - eta_d[1]), self.nu[1], rospy.get_time())
            tau_1 = math.cos(self.eta[5]) * tau_1_ned + math.sin(self.eta[5]) * tau_2_ned 
            tau_2 = - math.sin(self.eta[5]) * tau_1_ned + math.cos(self.eta[5]) * tau_2_ned
            tau = [tau_1, tau_2, tau_3, tau_4, tau_5, tau_6]

        elif self.control_type == 'course_control':
            if not self.control_type == self.prev_control_type:
                self.surge_speed_controller.initialize(rospy.get_time())
                self.sway_speed_controller.initialize(rospy.get_time())
            tau_1_ned_ref_ff = self.surge_sub_system.m * eta_dot_d[0]
            tau_1_ned = self.surge_speed_controller.regulate((self.nu[0] - eta_d[0]), 0, rospy.get_time())
            tau_2_ned_ref_ff = self.surge_sub_system.m * eta_dot_d[0]
            tau_2_ned = self.sway_speed_controller.regulate((self.nu[1] - eta_d[1]), 0, rospy.get_time())
            tau_1 = math.cos(self.eta[5]) * tau_1_ned + math.sin(self.eta[5]) * tau_2_ned
            tau_2 = - math.sin(self.eta[5]) * tau_1_ned + math.cos(self.eta[5]) * tau_2_ned
            tau = [tau_1, tau_2, tau_3, tau_4, tau_5, tau_6]

        else:
            tau = [0, 0, 0, 0, 0, 0]
        self.prev_control_type = self.control_type
        return tau

    def publish_control_forces(self):
        rate = rospy.Rate(self.controller_frequency)
        while not rospy.is_shutdown():
            try:
                self.get_state_estimates()
                eta_d, eta_dot_d, eta_ddot_d = self.generate_trajectory()
                tau = self.calculate_control_forces(eta_d, eta_dot_d, eta_ddot_d)
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
