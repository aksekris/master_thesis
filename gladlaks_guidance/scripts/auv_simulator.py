#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
import numpy as np
from quadratic_programming import quadprog_solve_qp
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt


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
        self.M = np.array(M_RB) + np.array(M_A)
        d = rospy.get_param("/auv_dynamics/D")
        self.D = -np.diag(d)
        self.m = rospy.get_param("/physical/mass_kg")
        r_g = rospy.get_param("/physical/center_of_mass")
        self.r_g = [-x for x in r_g]
        r_b = rospy.get_param("/physical/center_of_buoyancy")
        self.r_b = [-x for x in r_b]
        self.rho = rospy.get_param("/physical/water_density")
        self.Delta = rospy.get_param("/physical/volume")

        # Initialize the quadratic program
        duty_cycle_limits = rospy.get_param("/thrusters/duty_cycle_limits")
        propeller_revolution_limits = [(x - 1500)/0.1 for x in duty_cycle_limits]
        rotor_constant = rospy.get_param("/thrusters/rotor_constant")
        control_input_limits = [rotor_constant*abs(x)*x for x in propeller_revolution_limits]
        self.u_min = control_input_limits[0]
        self.u_max = control_input_limits[1]
        w = rospy.get_param("/auv_simulator/w")
        input_matrix = rospy.get_param("/thrusters/input_matrix")
        input_matrix_arr = np.array(input_matrix)
        self.pseudo_inv_input_matrix = np.dot(input_matrix_arr.T, np.linalg.inv(np.dot(input_matrix_arr, input_matrix_arr.T)))
        W = np.diag(w)
        self.P = np.block([[W*1.000001, -W], [-W, W*1.000001]]) # Cheat to make P positive definite
        self.c = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.A = np.block([[self.pseudo_inv_input_matrix, -self.pseudo_inv_input_matrix], [-self.pseudo_inv_input_matrix, self.pseudo_inv_input_matrix]])

        # Remove when using callbacks
        self.eta = [0, 0, 0, 0, 0, 0, 1] # Placeholder
        self.nu = [0, 0, 0, 0, 0, 0] # Placeholder
        self.eta_ref = [200, 1, 1, 0, 0, 1, 0]
        self.nu_ref = [0, 0, 0, 0, 0, 0]
        self.control_mode = 'dp_control'

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
        N = ns*freq
        h = 1/float(freq)
        # Option for adding ocean currents to the simulation (todo)
        eta_c_dot = [0, 0, 0, 0, 0, 0] # Irrotational

        # Initial states
        nu = self.nu
        eta = self.eta
        eta = np.array([0, 0, 0, 0, 0, 0, 1]) # Placeholder
        R = rotation_matrix_from_quat(eta[3:])
        T = T_from_quat(eta[3:])
        J = get_J(R,T)
        eta_dot = np.dot(J, self.nu)
        eta_ref = self.eta_ref

        # Initialize datasaving
        eta_d = np.zeros((int(N), 7))
        eta_dot_d = np.zeros((int(N), 7))
        nu_d = np.zeros((int(N), 6))
        nu_dot_d = np.zeros((int(N), 6))

        plot = []

        # Simulate the AUV
        for i, t in enumerate(list(np.linspace(t, t+ns, N))):

            # Unit quaternion normalization
            eta[3:] = unit_quaternion_normalization(eta[3:])

            R = rotation_matrix_from_quat(eta[3:])
            T = T_from_quat(eta[3:])
            J = get_J(R,T)

            gvect = get_gvect(self.m, self.rho, self.Delta, self.r_g, self.r_b, R)

            if any(x!=0 for x in eta_c_dot):
                nu_c = np.dot(np.block([[R.T, np.zeros((3, 3))], [np.zeros((3, 3)), R.T]]), eta_c_dot)
                nu_r = nu - nu_c
            else:
                nu_r = nu
                nu_c_dot = [0, 0, 0, 0, 0, 0]

            # Nonlinear PD-controller
            if self.control_mode == 'dp_control':
                # Create euler angle environment
                eta_euler = np.zeros(6)
                eta_euler[0:3] = eta[0:3]
                eta_euler[3:] = euler_from_quaternion(eta[3:])
                T_euler = T_from_eul(eta_euler[3:])
                J_euler = get_J(R,T_euler)
                eta_ref_euler = np.zeros(6)
                eta_ref_euler[0:3] = eta_ref[0:3]
                eta_ref_euler[3:] = euler_from_quaternion(eta_ref[3:])
                eta_dot_euler = eta_dot[0:6]
                eta_dot_euler[3:] = euler_from_quaternion(eta_dot[3:])
                # Virtual controller gains
                K_p = np.diag([2, 2, 20, 10, 10, 10]) # Ask Fossen
                K_d = np.diag([6, 6, 10, 0, 0, 0]) # Ask Fossen
                eta_euler_error = [a - b for a, b in zip(eta_euler, eta_ref_euler)]
                for k, eul in enumerate(eta_euler_error[3:]):
                    eta_euler_error[k+3] = ssa(eul)
                tau_eta = np.dot(K_p, eta_euler_error) + np.dot(K_d, eta_dot_euler)
                tau_unb = gvect - np.dot(np.linalg.inv(J_euler), tau_eta) 

            # Calculate control inputs
            u_unb = np.dot(self.pseudo_inv_input_matrix, tau_unb)
            
            # Claculate feasible control forces
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
            eta_dot = np.dot(J, nu)

            # Save trajectories
            eta_d[i] = eta
            nu_d[i] = nu 
            nu_dot_d[i] = nu_dot
            eta_dot_d[i] = eta_dot

            # Integration using Euler method
            eta = euler2(eta_dot, eta, h)
            nu = euler2(nu_dot, nu, h)


        return eta_d, nu_d, nu_dot_d

def skew_symmetric(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def euler2(x_dot, x, h):
    return x + x_dot*h

def unit_quaternion_normalization(q):
    offset = np.linalg.norm(q)
    return [x/offset for x in q]

def rotation_matrix_from_quat(Q):

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
    return -gvect

def T_from_quat(q):
    return 0.5*np.array([[ q[3], -q[2],  q[1]],
            [ q[2],  q[3], -q[0]],
            [-q[1],  q[0],  q[3]], 
            [-q[0], -q[1], -q[2]]])

def T_from_eul(eul):
    return np.array([[1, np.sin(eul[0])*np.tan(eul[1]), np.cos(eul[0])*np.tan(eul[1])], [0, np.cos(eul[0]), -np.sin(eul[0])], [0, np.sin(eul[0])/np.cos(eul[1]), np.cos(eul[0])]/np.cos(eul[1])])

def get_J(R,T):
    if T.shape == (4,3):
        return np.block([[R, np.zeros((3,3))], [np.zeros((4,3)), T]])
    elif T.shape == (3,3):
        return np.block([[R, np.zeros((3,3))], [np.zeros((3,3)), T]])

def ssa(error_angle):
    if error_angle < -np.pi or error_angle >= np.pi:
        # Map angle to [-pi, pi)
        error_angle = (error_angle + np.pi) % (2 * np.pi) - np.pi
    return error_angle

if __name__ == '__main__':
    auv_simulator = AUVSimulator()
    t = 0
    ns = 20
    freq = 100
    eta_d, nu_d, nu_dot_d = auv_simulator.simulate(t, ns, freq)

    x = []
    y = []
    z = []
    eul = []
    roll = []
    pitch = []
    yaw = []
    real = []
    
    for eta in eta_d:
        eul = (euler_from_quaternion(eta[3:]))
        x.append(eta[0])
        y.append(eta[1])
        z.append(eta[2])
        roll.append(eul[0]) 
        pitch.append(eul[1]) 
        yaw.append(eul[2]) 
    '''
    for nu in eta_dot_d:
        x.append(nu[0])
        y.append(nu[1])
        z.append(nu[2])
        roll.append(nu[3]) 
        pitch.append(nu[4]) 
        yaw.append(nu[5])
        real.append(nu[6])
    '''
    import matplotlib.pyplot as plt

    plt.plot(x)
    plt.ylabel('Position')
    plt.show()