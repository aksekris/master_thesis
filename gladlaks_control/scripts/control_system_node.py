#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
import math
import numpy as np
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped, AccelStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, euler_matrix
from functions import quaternion_to_rotation_matrix, LowPassFilter, MassDamperSpringSystem, PIDController, MassDamperSpring, FirstOrderSystem, HeadingAutopilot
from gladlaks_control.msg import ReferenceTrajectoryStamped


class ControlSystem:
    def __init__(self):
        
        # Initialize the ROS-node
        rospy.init_node('control_system')
        while rospy.get_time() == 0:
            continue
        rospy.Subscriber('/eskf_localization/pose', Odometry, self.pose_callback)
        rospy.Subscriber('/gladlaks/guidance_system/reference_trajectory', ReferenceTrajectoryStamped, self.reference_trajectory_callback)
        self.pub = rospy.Publisher('/gladlaks/thruster_manager/input_stamped', WrenchStamped, queue_size=1)
        self.rate = rospy.Rate(rospy.get_param("/control_system/frequency"))
        
        self.get_pose = False

        # Initialize the first pose hold
        self.reference_trajectory = ReferenceTrajectoryStamped()
        self.reference_trajectory.header.frame_id = 'dp_hold'
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 1
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        twist = TwistStamped()
        accel = AccelStamped
        self.reference_trajectory.poses.append(pose)
        self.reference_trajectory.twists.append(twist)
        self.reference_trajectory.accels.append(accel)
        self.prev_control_type = 'none'
        self.traj_index = 0

        # Download the AUV model
        M_RB = rospy.get_param("/auv_dynamics/M_RB")
        M_A = rospy.get_param("/auv_dynamics/M_A")
        self.M = np.array(M_RB) + np.array(M_A)
        d = rospy.get_param("/auv_dynamics/D")
        self.D = -np.diag(d)
        self.m = rospy.get_param("/physical/mass_kg")
        r_g = rospy.get_param("/physical/center_of_mass")
        self.r_g = [-x for x in r_g]
        r_b = rospy.get_param("/physical/center_of_buoyancy")
        self.r_b = [-x for x in r_b]
        self.rho = rospy.get_param("/physical/water_density")
        self.Delta = rospy.get_param("/physical/volume")

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

        # Initialize the heave dp controller
        m = M_RB[2][2]+M_A[2][2]
        d = -rospy.get_param("/auv_dynamics/D")[2]
        k = 0 
        self.heave_sub_system = MassDamperSpring(m, d, k)
        omega_b = rospy.get_param("/control_system/depth_controller/control_bandwidth")
        zeta = rospy.get_param("/control_system/depth_controller/relative_damping_ratio")
        tau_sat = rospy.get_param("/control_system/depth_controller/torque_saturation_limit")
        K_p, K_d, K_i = self.heave_sub_system.pid_pole_placement_algorithm(omega_b, zeta)
        self.heave_dp_controller = PIDController(K_p, K_d, K_i, tau_sat)

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
        
    def pose_callback(self, msg):
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

    def reference_trajectory_callback(self, msg):
        self.reference_trajectory = msg
        self.traj_index = 0

    def calculate_control_forces(self):
        # todo

        # Controllers: 
        #   surge, sway, heave, roll, pitch, yaw-dp
        #   surge, sway, heave-speed
        #
        # Control modes: 
        #   dp_hold: eta_d | surge, sway, heave, roll, pitch, yaw-dp
        #   dp_move: eta_d, nu_d, (nu_dot_d) | surge, sway, heave, roll, pitch, yaw-dp
        #   course: eta_d[3:], nu_d, (nu_dot_d) | surge, sway, heave-speed, roll, pitch, yaw-dp
        #   horizontal_course: eta_d[2:], nu_d, (nu_dot_d) | 
        #   vertical_course: eta[0:2 and 3:], nu_d, (nu_dot_d)

        # Get the current reference trajectory

        if self.traj_index == len(self.reference_trajectory.poses):
            self.traj_index += -1
            control_mode = 'dp_hold' 
        else:
            control_mode = self.reference_trajectory.header.frame_id

        pose = self.reference_trajectory.poses[self.traj_index].pose
        twist = self.reference_trajectory.twists[self.traj_index].twist
        accel = self.reference_trajectory.accels[self.traj_index].accel
        self.traj_index += 1

        euler_angles = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        R = euler_matrix(0, 0, euler_angles[2])  # NB! Assumption that roll and pitch are zero for robustness, alternatively R = euler_matrix(euler_angles)
        R = R[:3, :3]
        '''
        eta_d = extract_from_pose(pose)
        nu_d = extract_from_twist(twist)
        nu_dot_d = extract_from_accel(accel)
        '''
        # todo: figure out about switching between controllers
        # if control_mode != self.prev_control_mode:
            # Reinitialize the controllers if the control mode has changed
            
        self.surge_dp_controller.initialize(rospy.get_time())
        self.sway_dp_controller.initialize(rospy.get_time())
        self.heading_controller.initialize(rospy.get_time())
        self.heave_dp_controller.initialize(rospy.get_time())

        if control_mode == 'dp_hold':
            eta_d = extract_from_pose(pose)
            x_body_err, y_body_err, z_body_err = np.dot(R.T, [a - b for a, b in zip(self.eta[:3], eta_d[:3])])
            tau_1 = self.surge_dp_controller.regulate(x_body_err, self.nu[0], rospy.get_time())
            tau_2 = self.sway_dp_controller.regulate(y_body_err, self.nu[1], rospy.get_time())
            tau_3 = self.heave_dp_controller.regulate(z_body_err, self.nu[2], rospy.get_time())
            tau_4 = 0
            tau_5 = 0
            tau_6 = self.heading_controller.calculate_control_torque((self.eta[5] - eta_d[5]), self.nu[5], rospy.get_time())

        if control_mode == 'dp_move':
            eta_d = extract_from_pose(pose)
            nu_d = extract_from_twist(twist)
            nu_dot_d = extract_from_accel(accel)
            x_body_err, y_body_err, z_body_err = np.dot(R.T, self.eta[:3] - eta_d[:3])
            tau_1_ref_ff = self.D[0][0] * nu_d[0] + self.M[0][0] * nu_dot_d[0]
            tau_1 = tau_1_ref_ff + self.surge_dp_controller.regulate(x_body_err, self.nu[0], rospy.get_time())
            tau_2_ref_ff = self.D[1][1] * nu_d[1] + self.M[1][1] * nu_dot_d[1]
            tau_2 = tau_1_ref_ff + self.surge_dp_controller.regulate(y_body_err, self.nu[1], rospy.get_time())
            tau_3_ref_ff = self.D[2][2] * nu_d[2] + self.M[2][2] * nu_dot_d[2]
            tau_3 = tau_3_ref_ff + self.heave_dp_controller.regulate(z_body_err, self.nu[2], rospy.get_time())                
            tau_4 = 0
            tau_5 = 0
            tau_6_ref_ff = self.D[5][5] * nu_d[5] + self.M[5][5] * nu_dot_d[5]
            tau_6 = tau_6_ref_ff + self.heading_controller.calculate_control_torque((self.eta[5] - eta_d[5]), self.nu[5], rospy.get_time())
   
        if control_mode == 'dp_control':
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
        self.prev_control_mode = control_mode
        return tau

    def publish_control_forces(self):
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

def extract_from_accel(accel):
    linear = (accel.linear.x, accel.linear.y, accel.linear.z)
    angular = (accel.angular.x, accel.angular.y, accel.angular.z)
    return [linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]]

if __name__ == '__main__':
    try:
        node = ControlSystem()
        node.publish_control_forces()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
