#!/usr/bin/env python 
# Written by Aksel Kristoffersen

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('simulated_navigation')
    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/gladlaks/base_link_ned', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        

        print(trans)

        rate.sleep()