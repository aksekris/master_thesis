#!/usr/bin/env python
# Written by Aksel Kristoffersen

import numpy as np

def get_gvect(m, g, r_g, r_b, rho, volume, eul):
    gvect = np.zeros(6)
    W = m*g
    B = rho*g*volume
    R = R_from_eul(eul)
    f_g = np.dot(R.T, [0, 0, W])
    f_b = np.dot(R.T, [0, 0, -B])
    gvect[0:3] = f_g + f_b
    gvect[3:] = np.cross(r_g, f_g) + np.cross(r_b, f_b)
    return -gvect

def get_M_RB(m, r_g, inertia):
    M_11 = m*np.eye(3)
    M_12 = -m*skew(r_g)
    M_21 = m*skew(r_g)
    M_22 = np.array(inertia) - m*skew(r_g)**2
    return np.block([[M_11, M_12], [M_21, M_22]])

def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def R_x(rot):
    np.array([[1, 0, 0], 
              [0, np.cos(rot), -np.sin(rot)], 
              [0, np.sin(rot), np.cos(rot)]])

def R_y(rot):
    np.array([[np.cos(rot), 0, np.sin(rot)], 
              [0, 1, 0], 
              [-np.sin(rot), 0, np.cos(rot)]])

def R_z(rot):
    np.array([[np.cos(rot), -np.sin(rot), 0], 
              [np.sin(rot), np.cos(rot), 0], 
              [0, 0, 1]])

def R_path(chi_p, gamma_p):
    return np.dot(R_y(gamma_p), R_z(chi_p))

def R_from_eul(eul):
    return np.array([[np.cos(eul[2])*np.cos(eul[1]), -np.sin(eul[2])*np.cos(eul[0]) + np.cos(eul[2])*np.sin(eul[1])*np.sin(eul[0]), np.sin(eul[2])*np.sin(eul[0]) + np.cos(eul[2])*np.cos(eul[0])*np.sin(eul[1])],
                    [np.sin(eul[2])*np.cos(eul[1]), np.cos(eul[2])*np.cos(eul[0]) + np.sin(eul[0])*np.sin(eul[1])*np.sin(eul[2]), -np.cos(eul[2])*np.sin(eul[0]) + np.sin(eul[1])*np.sin(eul[2])*np.cos(eul[0])],
                    [-np.sin(eul[1]), np.cos(eul[1])*np.sin(eul[0]), np.cos(eul[1])*np.cos(eul[0])]])

def T_from_eul(eul):
    return np.array([[1, np.sin(eul[0])*np.tan(eul[1]), np.cos(eul[0])*np.tan(eul[1])], [0, np.cos(eul[0]), -np.sin(eul[0])], [0, np.sin(eul[0])/np.cos(eul[1]), np.cos(eul[0])]/np.cos(eul[1])])

def J_from_eul(eul):
    return np.block([[R_from_eul(eul), np.zeros((3,3))], [np.zeros((3,3)), T_from_eul(eul)]])



def euler2(dot_x, x, h):
    return x + dot_x*h