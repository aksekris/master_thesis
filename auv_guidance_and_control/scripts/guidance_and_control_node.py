#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import WrenchStamped, PoseStamped
from path import Path as Path1
from auv_model import AUVModel
from actuator_model import ActuatorModel
from control_system import DPControlSystem
from auv_simulator import AUVSimulator
from virtual_target import VirtualTarget

class GuidanceAndControlNode:
    def __init__(self):
        
        # Initialize the ROS-node
        rospy.init_node('guidance_and_control_system')
        while rospy.get_time() == 0:
            continue
        rospy.Subscriber('/eskf_localization/pose', Odometry, self.navigation_callback)
        self.pub = rospy.Publisher('/beluga/thruster_manager/input_stamped', WrenchStamped, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.get_pose = False

        m = rospy.get_param("/physical/mass_kg")
        r_g = rospy.get_param("/physical/center_of_mass")
        r_b = rospy.get_param("/physical/center_of_buoyancy")
        inertia = np.array(rospy.get_param("/physical/inertia"))
        volume = rospy.get_param("/physical/volume")
        M_A = np.array(rospy.get_param("/auv_dynamics/M_A"))
        D = -np.diag(rospy.get_param("/auv_dynamics/D"))
        rho = rospy.get_param("/physical/water_density")
        g = 9.81
        self.auv_model = AUVModel(m, r_g, r_b, inertia, volume, M_A, D, rho=rho, g=g, dot_eta_c=[0,0,0,0,0,0])

        input_matrix = np.array(rospy.get_param("/thrusters/input_matrix"))
        rotor_time_constant = rospy.get_param("/thrusters/first_order_time_constant")
        u_max = 31.5
        u_min = -31.5
        simulator_actuator_model = ActuatorModel(input_matrix, rotor_time_constant, u_max, u_min)

        M = self.auv_model.M
        D = self.auv_model.D
        omega_b = np.array(rospy.get_param("/guidance_and_control_system/control_bandwidth"))
        zeta = np.array(rospy.get_param("/guidance_and_control_system/relative_damping"))
        self.dp_control_system = DPControlSystem(M, D, omega_b, zeta)

        omega_b_simulator = [x*0.5 for x in omega_b]
        simulator_control_system = dp_control_system = DPControlSystem(M, D, omega_b_simulator, zeta)
        w = np.array(rospy.get_param("/guidance_and_control_system/control_forces_weights"))
        self.reference_model = AUVSimulator(self.auv_model, simulator_actuator_model, simulator_control_system, w)
        eta = [0, 0, 0.5, 0, 0, 0] 
        nu = [0, 0, 0, 0, 0, 0]
        dot_nu = [0, 0, 0, 0, 0, 0]
        tau = [0, 0, 0, 0, 0, 0]
        self.reference_model.set_initial_conditions(eta, nu, dot_nu, tau, rospy.get_time())

        self.publish_rate = rospy.get_param("/guidance_and_control_system/publish_rate")
        self.rate = rospy.Rate(self.publish_rate)
        self.mode = 'path_following'

        
        # Add whatever mode
        if self.mode == 'path_following':
            vt_u_max = 31.5/20
            vt_u_min = -31.5/20
            vt_actuator_model = ActuatorModel(input_matrix, rotor_time_constant, vt_u_max, vt_u_min)
            self.path = Path1()
            waypoints = [[0, 0, 0.5], [0, 0, 1], [1, 0, 1.5], [0, 0, 2], [-0.5, 0, 1.5], [-1.5, 0, 1.5], [-1, 0, 0.5], [0, 0, 0.5]]
            self.path.generate_G0_path(waypoints)
            self.path_following_controller = VirtualTarget(self.path, self.auv_model, vt_actuator_model, simulator_control_system)

            # Publish the path for vizualization purposes
            publish_path_once(self.path)

            # Broadcast virtual target for vizualization purposes
            
        elif self.mode == 'pose_hold':
            self.eta_r = [0, 0, 0.5, 0, 0, 0]
        

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
                # Computation
                self.get_state_estimates()
                if self.mode == 'path_following':
                    eta_r, nu_r, dot_nu_r = self.path_following_controller.generate_reference_trajectories(self.eta, self.nu, rospy.get_time())
                elif self.mode == 'pose_hold':
                    eta_r = self.eta_r
                    nu_r = [0, 0, 0, 0, 0, 0]
                    dot_nu_r = [0, 0, 0, 0, 0, 0]
                eta_d, nu_d, dot_nu_d = self.reference_model.generate_trajectory_for_dp(rospy.get_time(), 1, 1/float(self.publish_rate), eta_r, nu_ref=nu_r)
                tau_c = self.auv_model.get_gvect([0, 0, self.eta[5]]) + self.dp_control_system.pid_regulate(self.eta, self.nu, eta_d[0], nu_d[0], dot_nu_d[0], rospy.get_time())

                # Publishing
                p = eta_r[:3]
                eul = eta_r[3:]
                q = quaternion_from_euler(eul[0], eul[1], eul[2])
                self.br.sendTransform((p[0], p[1], p[2]), 
                                (q[0], q[1], q[2], q[3]),
                                rospy.Time.now(),
                                "/virtual_target",
                                "/world")
                msg = create_wrenchstamped_msg(tau_c, rospy.get_rostime())
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

def publish_path_once(path):
    path_pub = rospy.Publisher('/beluga/guidance_and_control_system/path', Path, queue_size=1)
    msg = Path()
    msg.header.frame_id = '/world'
    for i in range(len(path.path)):
        for j in list(np.linspace(0, 1, 50)):
            p = path.path[i](j)
            psi = path.chi_p[i](j)
            q = quaternion_from_euler(0, 0, psi)
            pose = PoseStamped()
            pose.header.frame_id = '/world'
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            msg.poses.append(pose)
    rate = rospy.Rate(10)
    ctrl_c = False
    while not ctrl_c:
        connections = path_pub.get_num_connections()
        if connections > 0:
            path_pub.publish(msg)
            ctrl_c = True
        else:
            rate.sleep()

def create_wrenchstamped_msg(tau, t):
    msg = WrenchStamped()
    msg.header.stamp = t
    msg.header.frame_id = "beluga/base_link_ned"
    msg.wrench.force.x = tau[0]
    msg.wrench.force.y = tau[1]
    msg.wrench.force.z = tau[2]
    msg.wrench.torque.x = tau[3]
    msg.wrench.torque.y = tau[4]
    msg.wrench.torque.z = tau[5]
    return msg

if __name__ == '__main__':
    try:
        node = GuidanceAndControlNode()
        node.publish_control_forces()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

