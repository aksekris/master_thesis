#!/usr/bin/env python
# Written by Aksel Kristoffersen
# Documentation can be found in my master's thesis, chapter __ and __

import numpy as np
import pandas as pd
from scipy.optimize import minimize
from functions import euler2, J_from_eul
from path import Path
from auv_model import AUVModel
from actuator_model import ActuatorModel
from control_system import DPControlSystem

class VirtualTarget():
    def __init__(self, Path, AUVModel, ActuatorModel, DPControlSystem, heading_mode='path_dependent_heading', dot_s_bounds=None, point=[0,0], current_velocity=[0,0,0,0,0,0]):
        self.path = Path
        self.auv_model = AUVModel
        self.actuator_model = ActuatorModel
        self.dp_control_system = DPControlSystem

        self.dot_s = 0
        self.path_index = 0
        self.varpi = 0
        self.t = 0

        self.heading_mode = heading_mode
        if self.heading_mode == 'point_dependent_heading':
            self.point = point
        self.dot_s_bounds = dot_s_bounds
        self.dot_eta_c = current_velocity
        self.update_states()

    def update_states(self):
        p = self.path.path[self.path_index](self.varpi)
        chi_p = self.path.chi_p[self.path_index](self.varpi)
        dot_chi_p = lambda dot_s : kappa_h*np.cos(gamma_p)*dot_s
        gamma_p = self.path.gamma_p[self.path_index](self.varpi)
        dot_gamma_p = lambda dot_s : kappa_v * dot_s
        kappa_h = self.path.kappa_h[self.path_index](self.varpi)
        kappa_v = self.path.kappa_v[self.path_index](self.varpi)
        if self.heading_mode == 'path_dependent_heading':
            psi_t = chi_p
            dot_psi_t = lambda dot_s : kappa_h*np.cos(gamma_p)*dot_s
            ddot_psi_t = lambda dot_s : -kappa_h*kappa_v*np.sin(gamma_p)*dot_s**2
        if self.heading_mode == 'point_dependent_heading':
            kappa_o = 1/np.linalg.norm([self.point[0] - p[0], self.point[1] - p[1]])
            dot_kappa_o = lambda dot_s : 1/(np.cos(chi_p - psi_t)*np.cos(gamma_p)*dot_s)
            psi_t = np.arctan2(self.point[0] - p[0], self.point[1] - p[1])
            dot_psi_t = lambda dot_s : -kappa_o*np.sin(chi_p - psi_t)*dot_s
            ddot_psi_t = lambda dot_s : dot_s  # Placeholder

        self.eta_t = np.array(p + [0.0, 0.0, psi_t])
        self.nu_t = lambda dot_s : np.array([np.cos(chi_p - psi_t)*np.cos(gamma_p)*dot_s] + 
                    [np.sin(chi_p - psi_t)*np.cos(gamma_p)*dot_s] +
                    [-np.sin(gamma_p)*dot_s] +
                    [0] + 
                    [0] +
                    [dot_psi_t(dot_s)])
        self.dot_nu_t = lambda dot_s : np.array([-np.sin(chi_p-psi_t)*np.cos(gamma_p)*(dot_chi_p(dot_s) - dot_psi_t(dot_s))*dot_s - np.cos(chi_p-psi_t)*np.sin(gamma_p)*dot_gamma_p(dot_s)*dot_s] +
                        [np.cos(chi_p-psi_t)*np.cos(gamma_p)*(dot_chi_p(dot_s) - dot_psi_t(dot_s))*dot_s - np.sin(chi_p-psi_t)*np.sin(gamma_p)*dot_gamma_p(dot_s)*dot_s] +
                        [-np.cos(gamma_p)*dot_gamma_p(dot_s)*dot_s] +
                        [0] +
                        [0] +
                        [ddot_psi_t(dot_s)])

    def optimize_along_track_speed(self, eta, nu, dot_eta_c):
        tau_c = lambda dot_s : self.auv_model.get_gvect(eta[3:]) + self.dp_control_system.pd_regulate(eta, nu, self.eta_t, self.nu_t(dot_s), self.dot_nu_t(dot_s), dot_eta_c)
        f = lambda x : -x
        g_1 = lambda x : -(np.dot(self.actuator_model.pseudo_inv_input_matrix, tau_c(x)) - self.actuator_model.u_max)
        g_2 = lambda x : -(-np.dot(self.actuator_model.pseudo_inv_input_matrix, tau_c(x)) + self.actuator_model.u_min)

        x_0 = self.dot_s
        bnds = self.dot_s_bounds
        con1 = {'type': 'ineq', 'fun': g_1}
        con2 = {'type': 'ineq', 'fun': g_2}
        cons = [con1, con2]
        sol = minimize(f, x_0, method='SLSQP', bounds=bnds, constraints=cons)
        if sol.success:
            self.dot_s = sol.x[0]
        else:
            self.dot_s = 0
            print(sol)

    def simulate_path_variables(self, t):
        h = t - self.t
        dot_varpi = self.dot_s/self.path.length[self.path_index]
        self.varpi = euler2(dot_varpi, self.varpi, h)
        self.t = t
        if self.varpi > 1:
            if not self.path_index == len(self.path.path)-1:
                self.path_index += 1
                rest = self.varpi - 1
                self.varpi = rest * self.path.length[self.path_index-1]/self.path.length[self.path_index]
            else:
                self.varpi = 1
                self.dot_s = 0

    def generate_reference_trajectories(self, eta, nu, t, dot_eta_c=[0,0,0,0,0,0]):
        self.update_states()
        self.optimize_along_track_speed(eta, nu, dot_eta_c)
        self.simulate_path_variables(t)
        return self.eta_t, self.nu_t(self.dot_s), self.dot_nu_t(self.dot_s), self.dot_s

if __name__ == '__main__':
    waypoints = [[0, 0, 0], [1, 1, 1], [0, 1, 0], [0, 0, 0]]
    path = Path()
    path.generate_G0_path(waypoints)

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

    virtual_target = VirtualTarget(path, auv_model, actuator_model, dp_control_system)

    eta = [1,0,0,0,0,0]
    nu = [0.1,0,0,0,0,0]
    dot_eta_c = [0,0,0,0,0,0]

    data = {
            't': [],
            'x': [], 'y': [], 'z': [], 'psi': [], 'dot_s': []
            }

    h = 0.01
    N = 500
    t = 0
    for i in range(N):
        t = i*h
        eta_r, nu_r, dot_nu_r, dot_s = virtual_target.generate_reference_trajectories(eta, nu, t)
        eta = eta_r
        nu = nu_r

        data['t'].append(t)

        data['x'].append(eta_r[0])
        data['y'].append(eta_r[1])
        data['z'].append(eta_r[2])
        data['psi'].append(eta_r[5])
        data['dot_s'].append(dot_s)

    df = pd.DataFrame(data)
    df.to_csv('matlab/virtual_target_dynamics1.csv')