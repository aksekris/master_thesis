#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from reference_models import LowPassFilter, MassDamperSpringSystem

class GuidanceSystem:
    def __init__(self):

        rospy.init_node('guidance_system')
        while rospy.get_time() == 0:
            continue
        pose_sub = rospy.Subscriber('/gladlaks/navigation_system/pose', PoseStamped, self.pose_callback)
        twist_sub = rospy.Subscriber('/gladlaks/navigation_system/twist', TwistStamped, self.twist_callback)
        self.pub_pose = rospy.Publisher('/gladlaks/control_system/input_pose', PoseStamped, queue_size=1)
        self.pub_twist = rospy.Publisher('/gladlaks/control_system/input_twist', TwistStamped, queue_size=1)
        self.controller_frequency = rospy.get_param("/control_system/controller_frequency")
        

        # Initialize the reference model
        eta = rospy.get_param("/initial_conditions/auv/eta")
        nu = [0, 0, 0, 0, 0, 0]
        delta = [1, 1, 1, 1, 1, 1]
        omega = [4, 4, 4, 4, 4, 4]

        eta = [eta[5]]
        nu = [nu[5]]
        omega = [omega[5]]
        delta = [delta[5]]
        # vel_limits =
        # acc_limits = 
        self.low_pass_filter = LowPassFilter(eta, omega, rospy.get_time())
        self.mass_damper_spring_system = MassDamperSpringSystem(eta, nu, delta, omega, rospy.get_time())

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

    def get_state_estimates(self):
        self.get_eta = True
        self.get_nu = True
        while self.get_eta or self.get_nu:
            continue

    def generate_setpoint(self):
        x_r = 0
        y_r = 0
        z_r = 0
        roll_r = 0
        pitch_r = 0
        yaw_r = 2
        eta_r = [x_r, y_r, z_r, roll_r, pitch_r, yaw_r]
        nu_r = [None, None, None, None, None, None]
        return eta_r, nu_r

    def generate_trajectory(self):
        eta_r, nu_r = self.generate_setpoint()
        x = self.low_pass_filter.simulate([eta_r[5]], rospy.get_time())
        eta_d, nu_d = self.mass_damper_spring_system.simulate(x, rospy.get_time())
        eta_r[5] = eta_d[0]
        nu_r[5] = nu_d[0]
        return eta_r, nu_r
        
    def publish_trajectory(self):
        rate = rospy.Rate(self.controller_frequency)
        while not rospy.is_shutdown():
            try:
                self.get_state_estimates()
                eta_d, nu_r = self.generate_trajectory()
                rot = quaternion_from_euler(eta_d[3], eta_d[4], eta_d[5])
                trans = eta_d[0:3]
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.get_rostime()
                pose_msg.header.frame_id = '/world'
                pose_msg.pose.position.x = trans[0]
                pose_msg.pose.position.y = trans[1]
                pose_msg.pose.position.z = trans[2]
                pose_msg.pose.orientation.x = rot[0]
                pose_msg.pose.orientation.y = rot[1]
                pose_msg.pose.orientation.z = rot[2]
                pose_msg.pose.orientation.w = rot[3]
                self.pub_pose.publish(pose_msg)
                """
                twist_msg = TwistStamped()
                twist_msg.header.stamp = rospy.get_rostime()
                twist_msg.header.frame_id = '/gladlaks/base_link_ned'
                twist_msgt.twist.linear.x = lin[0]
                twist_msg.twist.linear.y = lin[1]
                twist_msg.twist.linear.z = lin[2]
                twist_msg.twist.angular.x = ang[0]
                twist_msg.twist.angular.y = ang[1]
                twist_msg.twist.angular.z = ang[2]
                self.pub_twist.publish()
                """
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

if __name__ == '__main__':
    try:
        node = GuidanceSystem()
        node.publish_trajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass