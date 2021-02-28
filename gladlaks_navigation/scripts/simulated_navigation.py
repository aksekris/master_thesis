#!/usr/bin/env python 
# Written by Aksel Kristoffersen

import rospy
import tf
from nav_msgs.msg import Odometry
import numpy as np

if __name__ == '__main__':
    pub = rospy.Publisher('/eskf_localization/pose', Odometry, queue_size=1)
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
            msg = Odometry()

            msg.header.stamp = rospy.get_rostime()
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

            pub.publish(msg)
            rate.sleep()
        except rospy.ROSInterruptException:
            pass