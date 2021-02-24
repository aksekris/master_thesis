#!/usr/bin/env python 
# Written by Aksel Kristoffersen

import rospy
import tf
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    pub = rospy.Publisher('/gladlaks/navigation_system/pose', PoseStamped, queue_size=1)
    rospy.init_node('simulated_navigation')
    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/gladlaks/base_link_ned', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        msg = PoseStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = '/world'
        msg.pose.position.x = trans[0]
        msg.pose.position.y = trans[1]
        msg.pose.position.z = trans[2]
        msg.pose.orientation.x = rot[0]
        msg.pose.orientation.y = rot[1]
        msg.pose.orientation.z = rot[2]
        msg.pose.orientation.w = rot[3]

        pub.publish(msg)

        rate.sleep()