#!/usr/bin/env python
# Written by Aksel Kristoffersen
# Documentation can be found in my master's thesis, chapter __ and __

import numpy as np


class Path:
    def __init__(self):
        self.path = []
        self.chi_p = []
        self.gamma_p = []
        self.kappa_h = []
        self.kappa_v = []
        self.length = []

    def generate_G0_path(self, waypoints):
        for i in range(len(waypoints)-1):
            v = [a - b for a, b in zip(waypoints[i+1], waypoints[i])]
            l = np.linalg.norm(v)
            chi_l = np.arctan2(v[1], v[0])
            gamma_l = -np.arctan2(v[2] , np.linalg.norm(v[:2]))
            self.straight_line_path_segment(waypoints[i], l, chi_l, gamma_l)

    #def generate_G1_path(waypoints):


    def straight_line_path_segment(self, p_0, l, chi_l, gamma_l):
        self.path.append(lambda varpi : [p_0[0] + l*varpi*np.cos(chi_l)*np.cos(gamma_l), 
                            p_0[1] + l*varpi*np.sin(chi_l)*np.cos(gamma_l), 
                            p_0[2] - l*varpi*np.sin(gamma_l)])
        self.chi_p.append(lambda varpi : chi_l)
        self.gamma_p.append(lambda varpi : gamma_l)
        self.kappa_h.append(lambda varpi : 0)
        self.kappa_v.append(lambda varpi : 0)
        self.length.append(l)

    def curved_path_segment(self, c, r_h, r_v, chi_0, chi_1, gamma_0, gamma_1):
        self.path.append(lambda varpi : [c[0] + r_h*np.cos(chi_0 + varpi*(chi_1-chi_0)), 
                            c[1] + r_h*np.sin(chi_0 + varpi*(chi_1-chi_0)), 
                            c[2] + np.sign(gamma_1-gamma_0)*r_v*np.cos(gamma_0 + varpi*(gamma_1-gamma_0))])
        self.chi_p.append(lambda varpi : chi_0 + varpi*(chi_1-chi_0))
        self.gamma_p.append(lambda varpi : gamma_0 + varpi*(gamma_1-gamma_0))
        self.kappa_h.append(lambda varpi : 1/r_h)
        self.kappa_v.append(lambda varpi : 1/r_v)
        self.length.append(np.linalg.norm)


if __name__ == '__main__':
    waypoints = [[0, 0, 0], [1, 0, 1], [0, 1, 0], [0, 0, 0]]
    path = Path()
    path.generate_G0_path(waypoints)
    #print(path.path[0](0.5))


