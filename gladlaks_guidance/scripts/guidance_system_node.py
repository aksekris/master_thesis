#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from reference_models import LowPassFilter, MassDamperSpringSystem

class GuidanceSystem:
    def __init__(self):

        rospy.init_node('guidance_system')
        while rospy.get_time() == 0:
            continue
        rospy.Subscriber('/eskf_localization/pose', Odometry, self.callback)
        self.pub = rospy.Publisher('/gladlaks/control_system/input_pose', Odometry, queue_size=1)
        self.rate = rospy.Rate(rospy.get_param("/control_system/frequency"))
        self.get_pose = False

    def callback(self, msg):
        if self.get_pose:
            self.eta = extract_eta_from_odom(msg)
            self.nu = extract_nu_from_odom(msg)
            self.get_pose = False
        else:
            pass

    def get_state_estimates(self):
        self.get_pose = True
        while self.get_pose:
            continue

    def generate_setpoint(self):
        x_r = 1
        y_r = 1
        z_r = 1
        roll_r = 0
        pitch_r = 0
        yaw_r = 3.14
        eta_r = [x_r, y_r, z_r, roll_r, pitch_r, yaw_r]
        nu_r = [0.2, 0.2, 0, 0, 0, 0]
        return eta_r, nu_r
        
    def publish_trajectory(self):
        while not rospy.is_shutdown():
            try:
                self.get_state_estimates()
                eta_d, nu_d = self.generate_setpoint()
                trans = eta_d[0:3]
                rot = quaternion_from_euler(eta_d[3], eta_d[4], eta_d[5])
                lin = nu_d[0:3]
                ang = nu_d[3:]
                msg = Odometry()

                msg.header.stamp = rospy.get_rostime()
                msg.child_frame_id = 'dp_control'
                msg.pose.pose.position.x = trans[0]
                msg.pose.pose.position.y = trans[1]
                msg.pose.pose.position.z = trans[2]
                msg.pose.pose.orientation.x = rot[0]
                msg.pose.pose.orientation.y = rot[1]
                msg.pose.pose.orientation.z = rot[2]
                msg.pose.pose.orientation.w = rot[3]
                msg.twist.twist.linear.x = lin[0]
                msg.twist.twist.linear.y = lin[1]
                msg.twist.twist.linear.z = lin[2]
                msg.twist.twist.angular.z = ang[0]
                msg.twist.twist.angular.z = ang[1]
                msg.twist.twist.angular.z = ang[2]
                
                self.pub.publish(msg)

                self.rate.sleep()
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
        node = GuidanceSystem()
        node.publish_trajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass