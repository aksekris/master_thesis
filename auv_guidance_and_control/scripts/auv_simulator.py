#!/usr/bin/env python
# Written by Aksel Kristoffersen

import numpy as np
import quadprog
from auv_model import AUVModel
from actuator_model import ActuatorModel
from control_system import DPControlSystem
from functions import euler2, J_from_eul

class AUVSimulator():
    def __init__(self, AUVModel, ActuatorModel, DPControlSystem, w):
        self.actuator_model = ActuatorModel
        self.auv_model = AUVModel
        self.dp_control_system = DPControlSystem

        self.tau = np.array([0, 0, 0, 0, 0, 0])
        self.eta = np.array([0, 0, 0, 0, 0, 0])
        self.nu = np.array([0, 0, 0, 0, 0, 0])
        self.dot_nu = np.array([0, 0, 0, 0, 0, 0])
        self.t = 0

        # Setup for standard form QP
        W = np.diag(w)
        self.P = np.block([[W*1.000001, -W], [-W, W*1.000001]]) # Small cheat to make P positive definite
        self.c = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.A = np.block([[self.actuator_model.pseudo_inv_input_matrix, -self.actuator_model.pseudo_inv_input_matrix], [-self.actuator_model.pseudo_inv_input_matrix, self.actuator_model.pseudo_inv_input_matrix]])

    def set_initial_conditions(self, eta, nu, dot_nu, tau, t):
        self.eta = eta
        self.nu = nu
        self.dot_nu = dot_nu
        self.tau = tau
        self.t = t

    def generate_feasible_control_forces(self, tau_unb):
        u_unb = np.dot(self.actuator_model.pseudo_inv_input_matrix, tau_unb)
        if any([x>self.actuator_model.u_max or x<self.actuator_model.u_min for x in u_unb]):
            # If any unfeasible control inputs, generate feasible control forces using QP
            b = np.block([self.actuator_model.u_max - u_unb, -self.actuator_model.u_min + u_unb])
            x = quadprog_solve_qp(self.P, self.c, self.A, b)
            tau_b = tau_unb + x[0:6] - x[6:12]
        else:
            tau_b = tau_unb
        tau_b[3] = 0 # Ensure no actuation in roll or pitch
        tau_b[4] = 0
        return tau_b

    def generate_trajectory_for_dp(self, t, N, h, eta_ref, nu_ref=[0,0,0,0,0,0], dot_nu_ref=[0,0,0,0,0,0]):
        # Initialize datasaving
        eta_d = np.zeros((int(N), 6))
        nu_d = np.zeros((int(N), 6))
        dot_nu_d = np.zeros((int(N), 6))

        # Simulate the AUV
        for i, t in enumerate(list(np.linspace(t, t+h*N, N))):
            
            J = J_from_eul(self.eta[3:])
            nu_c = np.dot(J.T, self.auv_model.dot_eta_c)
            nu_r = self.nu - nu_c

            # Simulator control law
            tau_unb = self.auv_model.get_gvect(self.eta[3:]) + self.dp_control_system.pd_regulate(self.eta, self.nu, eta_ref, nu_ref, dot_nu_ref, self.auv_model.dot_eta_c)
            tau_b = self.generate_feasible_control_forces(tau_unb)

            # Simulate the actuator dynamics
            dot_tau = (tau_b - self.tau)/self.actuator_model.rotor_time_constant
            self.tau = euler2(dot_tau, self.tau, h)
            # Simulate the AUV dynamics
            self.dot_nu = np.dot(np.linalg.inv(np.diag(np.diag(self.auv_model.M))), self.tau - np.dot(self.auv_model.D, nu_r) - self.auv_model.get_gvect(self.eta[3:]))
            self.nu = euler2(self.dot_nu, self.nu, h)
            dot_eta = np.dot(J, self.nu)
            self.eta = euler2(dot_eta, self.eta, h)

            eta_d[i], nu_d[i], dot_nu_d[i] = self.eta, self.nu, self.dot_nu
            #u_b = np.dot(self.control_allocation_system.pseudo_inv_input_matrix, tau_b) # Remove later

        return eta_d, nu_d, dot_nu_d

def quadprog_solve_qp(P, q, G=None, h=None, A=None, b=None):
    qp_G = .5 * (P + P.T)   # make sure P is symmetric
    qp_a = -q
    if A is not None:
        qp_C = -numpy.vstack([A, G]).T
        qp_b = -numpy.hstack([b, h])
        meq = A.shape[0]
    else:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
        meq = 0
    return quadprog.solve_qp(qp_G.astype(np.float), qp_a.astype(np.float), qp_C.astype(np.float), qp_b.astype(np.float), meq)[0]

if __name__ == '__main__':
    m = 30.9
    r_g = [0, 0, 0.02]
    r_b = [0, 0, -0.05]
    inertia = [[ 0.503217, 0.000204, -0.000526],
               [ 0.000204, 0.893449, 0.000038],
               [ -0.000526, 0.000038, 0.919819]]
    volume = 0.0295
    M_A = [[ 10.7727, 0, 0, 0, 0, 0], 
           [ 0, 10.7727, 0, 0, 0, 0], 
           [ 0, 0, 49.7679, 0, 0, 0], 
           [ 0, 0, 0, 1.0092, 0, 0], 
           [ 0, 0, 0, 0, 1.0092, 0], 
           [ 0, 0, 0, 0, 0, 0]]
    D = [[ -9.5909, 0, 0, 0, 0, 0], 
        [ 0, -9.5909, 0, 0, 0, 0], 
        [ 0, 0, -50.5595, 0, 0, 0], 
        [ 0, 0, 0, -13.3040, 0, 0], 
        [ 0, 0, 0, 0, -13.3040, 0], 
        [ 0, 0, 0, 0, 0, -5.1559]]
    auv_model = AUVModel(m, r_g, r_b, inertia, volume, M_A, D, rho=997, g=9.81, dot_eta_c=[0,0,0,0,0,0])

    input_matrix = [[-0.707,  0.000,  0.000,  0.707,  0.707,  0.000,  0.000, -0.707], 
                    [-0.707,  0.000,  0.000, -0.707,  0.707,  0.000,  0.000,  0.707],
                    [ 0.000,  1.000,  1.000,  0.000,  0.000,  1.000,  1.000,  0.000],
                    [ 0.046, -0.220, -0.220,  0.046, -0.046,  0.220,  0.220, -0.046],
                    [-0.046, -0.120,  0.120,  0.046,  0.046,  0.119, -0.119, -0.046],
                    [-0.324,  0.000,  0.000,  0.325, -0.325,  0.000,  0.000,  0.324]]
    rotor_time_constant = 0.2
    u_max = 31.5
    u_min = -31.5
    actuator_model = ActuatorModel(input_matrix, rotor_time_constant, u_max, u_min)

    omega_b = [1, 1, 1, 1, 1, 1]
    zeta = [1, 1, 1, 1, 1, 1]
    dp_control_system = DPControlSystem(auv_model.M, auv_model.D, omega_b, zeta)

    auv_simulator = AUVSimulator(auv_model, actuator_model, dp_control_system)

    eta = [0, 0, 0, 0, 0, 0]
    nu = [0, 0, 0, 0, 0, 0]
    dot_nu = [0, 0, 0, 0, 0, 0]
    tau = [0, 0, 0, 0, 0, 0]
    t = 0
    auv_simulator.set_initial_conditions(eta, nu, dot_nu, tau, t)



    N = 50*5
    h = 0.02
    eta_ref = [1, 0, 0, 0, 0, 0]
    eta_d, nu_d, dot_nu_d = auv_simulator.generate_trajectory_for_dp(t, N, h, eta_ref)
    
