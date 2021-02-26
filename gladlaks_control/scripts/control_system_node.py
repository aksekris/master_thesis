#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from tf.transformations import euler_from_quaternion
import threading

class ControlSystem:
    def __init__(self):
        rospy.init_node('control_system')
        rospy.Rate(10)
        self.pose_sub = rospy.Subscriber('/gladlaks/navigation_system/pose', PoseStamped, self.pose_callback)
        self.twist_sub = rospy.Subscriber('/gladlaks/navigation_system/twist', TwistStamped, self.twist_callback)
        self.pub = rospy.Publisher('/gladlaks/thruster_manager/input_stamped', WrenchStamped, queue_size=1)
        self.controller_frequency = rospy.get_param("/control_system/controller_frequency")
        self.get_eta = False
        self.eta = [None, None, None, None, None, None]
        self.get_nu = False
        self.nu = [None, None, None, None, None, None]

    def pose_callback(self, pose_msg):
        if self.get_eta:
            quaternions = pose_msg.pose.orientation
            euler_angles = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])
            position = (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
            self.eta = [position[0], position[1], position[2], euler_angles[0], euler_angles[1], euler_angles[2]]
            print(self.eta)
            self.get_eta = False
        else:
            pass

    def twist_callback(self, twist_msg):
        if self.get_nu:
            linear = (twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z)
            angular = (twist_msg.twist.angular.x, twist_msg.twist.angular.y, twist_msg.twist.angular.z)
            self.get_nu = False
        else:
            pass
    
    def get_state_estimates(self):
        self.get_eta = True
        self.get_nu = True
        while  self.get_eta or self.get_nu:
            continue

    def heading_autopilot(self):
        quaternions = self.pose.orientation
        euler_angles = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])
        return 0.5

    def publish_control_forces(self):
        rate = rospy.Rate(self.controller_frequency)
        while not rospy.is_shutdown():
            try:
                self.get_state_estimates()
                #msg = WrenchStamped()
                #msg.wrench.torque.z = self.heading_autopilot()
                rate.sleep()
                #self.pub.publish(msg)
                
            except rospy.ROSInterruptException:
                pass
        
if __name__ == '__main__':
    try:
        node = ControlSystem()
        worker = threading.Thread(target=node.publish_control_forces)
        worker.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
