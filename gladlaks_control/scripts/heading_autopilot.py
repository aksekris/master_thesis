#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
from geometry_msgs.msg import WrenchStamped
from pid_controller import PIDRegulator


def talker():
    pub = rospy.Publisher('/gladlaks/thruster_manager/input_stamped', WrenchStamped, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = WrenchStamped()
        msg.wrench.force.z = 30
        msg.wrench.torque.z = 0
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass