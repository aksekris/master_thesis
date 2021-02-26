#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
import threading

class MotionController:
    def __init__(self):
        rospy.init_node('motion_controller')
        rospy.Rate(10)
        pose_sub = rospy.Subscriber('/gladlaks/navigation_system/pose', PoseStamped, self.pose_callback)
        twist_sub = rospy.Subscriber('/gladlaks/navigation_system/twist', TwistStamped, self.twist_callback)
        self.pose = PoseStamped()
        self.twist = TwistStamped()

    def pose_callback(self, pose_msg):
        self.pose = pose_msg

    def twist_callback(self, twist_msg):
        pass
    
    def publish_control_forces(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                print(self.pose)
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        

if __name__ == '__main__':
    try:
        motion_controller_node = MotionController()
        worker = threading.Thread(target=motion_controller_node.publish_control_forces)
        worker.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
