#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from pid_controller import PIDRegulator
from tf.transformations import euler_from_quaternion
import numpy as np
import math

def callback(data):
    quaternions = data.pose.orientation
    euler_angles = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])
    psi = euler_angles[2]

    psi_d = math.pi
    e = ((psi-psi_d) + np.pi) % (2 * np.pi) - np.pi
    print(e)
    u = PID.regulate(e, rospy.get_time())

    msg = WrenchStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "gladlaks/base_link_ned"
    msg.wrench.torque.z = -u
    pub.publish(msg)

if __name__ == '__main__':
    try:
        PID = PIDRegulator(0.1, 0, 0, 1)
        rospy.init_node('heading_autopilot', anonymous=True)
        sub = rospy.Subscriber('/gladlaks/navigation_system/pose', PoseStamped, callback)
        pub = rospy.Publisher('/gladlaks/thruster_manager/input_stamped', WrenchStamped, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
