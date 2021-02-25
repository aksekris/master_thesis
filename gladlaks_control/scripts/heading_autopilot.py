#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from tf.transformations import euler_from_quaternion
from pid_controller import PIDController
from pid_pole_placement_algorithm import pid_pole_placement_algorithm

class HeadingAutopilot:
    """Heading autopilot based on PID pole-placement"""

    def __init__(self, m, d, k, omega_b, zeta, tau_sat):
        """Initialize the autopilot

        Args: 
            m           The mass coefficient
            d           The damper coefficient
            k           The spring coefficient
            omega_b     The control bandwidth
            zeta        The relative damping ratio
            tau_sat     The input torque saturation limit

        """
        (K_p, K_d, K_i) = pid_pole_placement_algorithm(m, d, k, omega_b, zeta)
        self.PID = PIDController(K_p, K_d, K_i, tau_sat)

    def calculate_control_torque(self, yaw_err, yaw_dt, t):
        """Calculate the control torque for heading control using a PID controller

        Args:
            yaw_err     The yaw error
            yaw_dt      The yaw derivative
            t           The time now

        Returns:
            float:      The control torque tau

        """
        if yaw_err < -np.pi or yaw_err >= np.pi:
            # Map angle to [-pi, pi)
            yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.p
        return self.PID.regulate(yaw_err, yaw_dt, t)
        

if __name__ == '__main__':
    pilot = HeadingAutopilot(1, 1, 1, 1, 1, 1)
    tau = pilot.calculate_control_torque(1, 1, 1)
    print(tau)

"""
    def __init__(self, iterable):
        self. = []
        self.__update(iterable)

    sub1 = rospy.Subscriber('/gladlaks/navigation_system/pose', PoseStamped, callback)
    sub2 = rospy.Subscriber('/gladlaks/navigation_system/twist', TwistStamped, callback)

    rate.sleep()

if __name__ == '__main__':
    rospy.init_node('heading_autopilot')
    try:
        HeadingAutopilot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


def callback(data):
    quaternions = data.pose.orientation
    euler_angles = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])
    psi = euler_angles[2]

    psi_d = math.pi
    e = ((psi-psi_d) + np.pi) % (2 * np.pi) - np.pi
    u = PID.regulate(e, rospy.get_time())
    msg = WrenchStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "gladlaks/base_link_ned"
    msg.wrench.torque.z = -u
    pub.publish(msg)

if __name__ == '__main__':
    try:
        omega_b = rospy.get_param("/control_system/heading_autopilot/control_bandwidth")
        zeta = rospy.get_param("/control_system/heading_autopilot/relative_damping_ratio")
        M_RB = rospy.get_param("/auv_dynamics/M_RB")
        M_A = rospy.get_param("/auv_dynamics/M_A")
        m = M_RB[5][5]+M_A[5][5]
        d = rospy.get_param("/auv_dynamics/D")[5]
        k = 0
        print(m, d, k, omega_b, zeta)
        (K_p, K_d, K_i) = PID_pole_placement_algorithm(m, d, k, omega_b, zeta)
        print(K_p, K_d, K_i)
        PID = PIDRegulator(K_p, 0, K_i, 44)
        rospy.init_node('heading_autopilot', anonymous=True)
        sub1 = rospy.Subscriber('/gladlaks/navigation_system/pose', PoseStamped, callback)
        sub2 = rospy.Subscriber('/gladlaks/navigation_system/twist', TwistStamped)
        pub = rospy.Publisher('/gladlaks/thruster_manager/input_stamped', WrenchStamped, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
"""