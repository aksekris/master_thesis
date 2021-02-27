#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped

class GuidanceSystem:
    def __init__(self):
        rospy.init_node('guidance_system')
        rospy.Rate(20)
        self.pub_pose = rospy.Publisher('/gladlaks/control_system/input_pose', PoseStamped, queue_size=1)
        self.pub_twist = rospy.Publisher('/gladlaks/control_system/input_twist', TwistStamped, queue_size=1)

    def generate_trajectory(self)
        
    def publish_trajectory(self)
        while not rospy.is_shutdown():
            try:
                = generate_trajectory()
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

                twist_msg = TwistStamped()
                twist_msg.header.stamp = rospy.get_rostime()
                twist_msg.header.frame_id = '/gladlaks/base_link_ned'
                twist_msgt.twist.linear.x = lin[0]
                twist_msg.twist.linear.y = lin[1]
                twist_msg.twist.linear.z = lin[2]
                twist_msg.twist.angular.x = ang[0]
                twist_msg.twist.angular.y = ang[1]
                twist_msg.twist.angular.z = ang[2]

                self.pub_pose.publish()
                self.pub_twist.publish()
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

if __name__ == '__main__':
    try:
        node = GuidanceSystem()
        node.publish_trajectory()
    except rospy.ROSInterruptException:
        pass