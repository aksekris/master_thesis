#!/usr/bin/env python 
# Written by Aksel Kristoffersen

import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped


if __name__ == '__main__':
    pub_pose = rospy.Publisher('/gladlaks/navigation_system/pose', PoseStamped, queue_size=1)
    pub_twist = rospy.Publisher('/gladlaks/navigation_system/twist', TwistStamped, queue_size=1)
    rospy.init_node('simulated_navigation')
    listener = tf.TransformListener()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/gladlaks/base_link_ned', rospy.Time(0))
            (lin,ang) = listener.lookupTwistFull( 
                '/gladlaks/base_link_ned',
                '/world', 
                '/gladlaks/base_link_ned', 
                (trans[0], trans[1], trans[2]),
                '/gladlaks/base_link_ned',
                rospy.Time(0),
                rospy.Duration(0.1)
                )
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        try:
            msg_pose = PoseStamped()
            msg_pose.header.stamp = rospy.get_rostime()
            msg_pose.header.frame_id = '/world'
            msg_pose.pose.position.x = trans[0]
            msg_pose.pose.position.y = trans[1]
            msg_pose.pose.position.z = trans[2]
            msg_pose.pose.orientation.x = rot[0]
            msg_pose.pose.orientation.y = rot[1]
            msg_pose.pose.orientation.z = rot[2]
            msg_pose.pose.orientation.w = rot[3]

            pub_pose.publish(msg_pose)
            
            msg_twist = TwistStamped()
            msg_twist.header.stamp = rospy.get_rostime()
            msg_twist.header.frame_id = '/gladlaks/base_link_ned'
            msg_twist.twist.linear.x = lin[0]
            msg_twist.twist.linear.y = lin[1]
            msg_twist.twist.linear.z = lin[2]
            msg_twist.twist.angular.x = ang[0]
            msg_twist.twist.angular.y = ang[1]
            msg_twist.twist.angular.z = ang[2]

            pub_twist.publish(msg_twist)

            rate.sleep()
        except rospy.ROSInterruptException:
            pass