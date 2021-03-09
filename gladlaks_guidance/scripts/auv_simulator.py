#!/usr/bin/env python
# Written by Aksel Kristoffersen

import numpy as np
from quadratic_programming import quadprog_solve_qp
from nav_msgs.msg import Path, Odometry

class AUVSimulator():
    def __init__(self):

        # Initialize the ROS node
        rospy.init_node('auv_simulator')
        while rospy.get_time() == 0:
            continue
        self.pub = rospy.Publisher('/gladlaks/guidance_system/auv_simulator/path', Path, queue_size=1)
        input_sub = rospy.Subscriber('/gladlaks/guidance_system/auv_simulator/input_reference', Odometry, self.input_callback)
        pose_sub = rospy.Subscriber('/eskf_localization/pose', Odometry, self.pose_callback)

        # Initialize the AUV model
        M_RB = rospy.get_param("/auv_dynamics/M_RB")
        M_A = rospy.get_param("/auv_dynamics/M_A")
        self.M = M_RB + M_A
        self.D = rospy.get_param("/auv_dynamics/D")
        self.m = rospy.get_param("/physical/mass_kg")
        self.r_g = -rospy.get_param("/physical/center_of_mass")
        self.r_b = -rospy.get_param("/physical/center_of_buoyancy")
        self.rho = rospy.get_param("/physical/water_density")
        self.Delta = rospy.get_param("/physical/volume")


        # Initialize the quadratic program
        duty_cycle_limits = rospy.get_param("/thrusters/duty_cycle_limits")
        propeller_revolution_limits = [(x - 1500)/0.1 for x in duty_cycle_limits]
        rotor_constant = rospy.get_param("/thrusters/rotor_constant")
        control_input_limits = [rotor_constant*abs(x)*x for x in propeller_revolution_limits]
        self.u_min = control_input_limits[0]
        self.u_max = control_input_limits[1]
        w = rospy.get_param("/auv_simulator/W")
        input_matrix = rospy.get_param("/thrusters/input_matrix")
        input_matrix_arr = np.array(input_matrix)
        self.pseudo_inv_input_matrix = np.dot(input_matrix_arr.T, np.linalg.inv(np.dot(input_matrix_arr, input_matrix_arr.T)))
        W = np.diag(w)
        self.P = np.block([[W*1.000001, -W], [-W, W*1.000001]]) # Cheat to make P positive definite
        self.c = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.A = np.block([[self.pseudo_inv_input_matrix, -self.pseudo_inv_input_matrix], [-self.pseudo_inv_input_matrix, self.pseudo_inv_input_matrix]])

        # Remove when using pose input
        self.eta = [0, 0, 0, 0, 0, 0, 1] # Placeholder
        self.nu = [0, 0, 0, 0, 0, 0] # Placeholder

    def input_callback(msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        self.eta_ref = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        v = msg.twist.twist.linear
        omeqa = msg.twist.twist.angular
        self.nu_ref = [v.x, v.y, v.z, omega.x, omega.y, omega.z]
        self.control_mode = msg.child_frame_id

    def pose_callback(msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        self.eta = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        v = msg.twist.twist.linear
        omeqa = msg.twist.twist.angular
        self.nu = [v.x, v.y, v.z, omega.x, omega.y, omega.z]

    def simulate(self, t, ns, freq):

        # self.control_mode = ['dp_control', 'horizontal_course_control', 'vertical_course_control']
        h = 1/freq
        N = ns*freq
        # Option for adding ocean currents to the simulation (todo)
        eta_c_dot = [0, 0, 0, 0, 0, 0] # Irrotational

        # Initial conditions
        nu = self.nu
        eta = self.eta

        # Initialize datasaving
        eta_d = np.zeros((N, 6))
        nu_d = np.zeros((N, 6))
        nu_dot_d = np.zeros((N, 6))



        # Simulate the AUV
        for i, t in enumerate(np.linspace(t, t+ns, N)):
            
            R = unit_quaternion_to_rotation_matrix(eta[3:])
            gvect = get_gvect(self.m, self.rho, self.Delta, self.r_g, self.r_b, R)
            J = 0
            if any(x!=0 for x in eta_c_dot):
                nu_c = np.dot(np.block([[R.T, np.zeros((3, 3))], [np.zeros((3, 3)), R.T]]), eta_c_dot)
                nu_r = nu - nu_c
            else:
                nu_r = nu
                nu_c_dot = [0, 0, 0, 0, 0, 0]

            # Nonlinear PD-controller
            if self.control_mode == 'dp_control':
                K_p = # Ask Fossen
                K_d = # Ask Fossen
                tau_pd = np.dot(K_p, eta - self.eta_ref) + np.dot(K_d, eta_dot)
                tau_unb = gvect - np.dot(J, tau_pd)

            # Calculate control inputs
            u_unb = np.dot(self.pseudo_inv_input_matrix, tau_unb)

            if any([x>self.u_max or x<self.u_min for x in u_unb]):
                # If any unfeasible control inputs, generate feasible control forces using quadratic programming
                b = np.block([self.u_max - u_unb, -self.u_min + u_unb])
                x = quadprog_solve_qp(self.P, self.c, self.A, b)
                tau_b = tau_unb + x[0:6] - x[6:12]
            else:
                tau_b = tau_unb

            # Simulate dynamics
            nu_r_dot = np.dot(np.linalg.inv(self.M), tau_b - np.dot(self.D, nu_r) - gvect) # Insert coriolis terms? Nonlinear damping? Ask Fossen
            nu_dot = nu_r_dot + nu_c_dot
            eta_dot = np.dot(J*nu)

            # Save trajectories
            eta_d[i] = eta
            nu_d[i] = nu
            nu_dot_d[i] = nu_dot

            # Integration using Euler method
            eta = euler2(eta_dot, eta, h)
            nu = euler2(nu_dot, nu, h)

            # Unit quaternion normalization
            eta[3:] = unit_quaternion_normalization(eta[3:])


        return eta_d, nu_d, nu_dot_d

def euler2(x_dot, x, h):
    # todo
    return x

def unit_quaternion_normalization(q):
    # todo
    return q

def unit_quaternion_to_rotation_matrix(Q):
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
    q0 = Q[3]
    q1 = Q[0]
    q2 = Q[1]
    q3 = Q[2]
     
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

def get_gvect(m, rho, Delta, r_g, r_b, R, g=9.81):
    gvect = np.zeros(6)
    W = m*g
    B = rho*g*Delta
    f_g = np.dot(R.T, [0, 0, W])
    f_b = np.dot(R.T, [0, 0, -B])

    gvect[0:3] = f_g + f_b
    gvect[3:] = np.cross(r_g, f_g) + np.cross(r_b, f_b)
    return gvect

if __name__ == '__main__':
    auv_simulator = AUVSimulator()
    t = 0
    ns = 3
    freq = 50
    auv_simulator.simulate(t, ns, freq)
    
    


