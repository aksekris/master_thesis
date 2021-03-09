#!/usr/bin/env python 
# Written by Aksel Kristoffersen

import rospy
import tf
from nav_msgs.msg import Odometry
import numpy as np

import numpy as np
 
def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

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
            #R = quaternion_rotation_matrix([rot[3], rot[0], rot[1], rot[2]])
            #lin = np.dot(R.T,lin)
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