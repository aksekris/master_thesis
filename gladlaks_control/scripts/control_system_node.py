#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
import threading
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from tf.transformations import euler_from_quaternion
from autopilots import HeadingAutopilot
from pid_controller import PIDController
from pid_pole_placement_algorithm import pid_pole_placement_algorithm


class ControlSystem:
    def __init__(self):

        rospy.init_node('control_system')
        while rospy.get_time() == 0:
            continue
        pose_sub = rospy.Subscriber('/gladlaks/navigation_system/pose', PoseStamped, self.pose_callback)
        twist_sub = rospy.Subscriber('/gladlaks/navigation_system/twist', TwistStamped, self.twist_callback)
        input_pose_sub = rospy.Subscriber('/gladlaks/control_system/input_pose', PoseStamped, self.input_pose_callback)
        self.pub = rospy.Publisher('/gladlaks/thruster_manager/input_stamped', WrenchStamped, queue_size=1)
        self.controller_frequency = rospy.get_param("/control_system/controller_frequency")
        self.get_eta = False
        self.eta = [None, None, None, None, None, None]
        self.eta_d = [0, 0, 0.5, 0, 0, 0] # Placeholder
        self.get_nu = False
        self.nu = [None, None, None, None, None, None]
        self.nu_d = [None, None, None, None, None, None] # Placeholder

        M_RB = rospy.get_param("/auv_dynamics/M_RB")
        M_A = rospy.get_param("/auv_dynamics/M_A")

        # Initialize the heading controller
        m = M_RB[5][5]+M_A[5][5]
        d = -rospy.get_param("/auv_dynamics/D")[5]
        k = 0
        omega_b = rospy.get_param("/control_system/heading_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/heading_controller/relative_damping_ratio")
        tau_sat = rospy.get_param("/control_system/heading_controller/torque_saturation_limit")
        (K_p, K_d, K_i) = pid_pole_placement_algorithm(m, d, k, omega_b, zeta)
        self.heading_controller = HeadingAutopilot(K_p, K_d, K_i, tau_sat, rospy.get_time())
        
        # Initialize the depth controller
        m = M_RB[2][2]+M_A[2][2]
        d = -rospy.get_param("/auv_dynamics/D")[2]
        k = 0  
        omega_b = rospy.get_param("/control_system/depth_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/depth_controller/relative_damping_ratio")
        tau_sat = rospy.get_param("/control_system/depth_controller/torque_saturation_limit")
        (K_p, K_d, K_i) = pid_pole_placement_algorithm(m, d, k, omega_b, zeta)
        self.depth_controller = PIDController(K_p, K_d, K_i, tau_sat, rospy.get_time())

    def pose_callback(self, pose_msg):
        if self.get_eta:
            quaternions = pose_msg.pose.orientation
            euler_angles = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])
            position = (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
            self.eta = [position[0], position[1], position[2], euler_angles[0], euler_angles[1], euler_angles[2]]
            self.get_eta = False
        else:
            pass

    def twist_callback(self, twist_msg):
        if self.get_nu:
            linear = (twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z)
            angular = (twist_msg.twist.angular.x, twist_msg.twist.angular.y, twist_msg.twist.angular.z)
            self.nu = [linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]]
            self.get_nu = False
        else:
            pass
    
    def input_pose_callback(self, input_pose_msg):
        quaternions = input_pose_msg.pose.orientation
        euler_angles = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])
        position = (input_pose_msg.pose.position.x, input_pose_msg.pose.position.y, input_pose_msg.pose.position.z)
        self.eta_d[5] = euler_angles[2]

    def desired_pose_callback(self):
        self.eta_d = [0, 0, 0, 0, 0, 0]

    def desired_twist_callback(self):
        self.nu_d = [0, 0, 0, 0, 0, 0]

    def get_state_estimates(self):
        self.get_eta = True
        self.get_nu = True
        while self.get_eta or self.get_nu:
            continue

    def calculate_control_forces(self):
        tau_1 = 0
        tau_2 = 0
        tau_3 = self.depth_controller.regulate((self.eta[2] - self.eta_d[2]), self.nu[2], rospy.get_time(), u_ff=23)                
        tau_4 = 0
        tau_5 = 0
        tau_6 = self.heading_controller.calculate_control_torque((self.eta[5] - self.eta_d[5]), self.nu[5], rospy.get_time())
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
                msg.wrench.force.z = tau[2]
                msg.wrench.torque.z = tau[5]
                rate.sleep()
                self.pub.publish(msg)
            except rospy.ROSInterruptException:
                pass
        
if __name__ == '__main__':
    try:
        node = ControlSystem()
        worker = threading.Thread(target=node.publish_control_forces)
        worker.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
