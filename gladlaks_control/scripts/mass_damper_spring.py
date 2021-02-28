#!/usr/bin/env python
# Written by Aksel Kristoffersen

from math import sqrt

class MassDamperSpring(object):
    """A mass-damper-spring object"""
    def __init__(self, m, d, k):
        self.m = m
        self.d = d
        self.k = k

    def pid_pole_placement_algorithm(self, omega_b, zeta):
        """PID pole-placement based on algorithm described in Fossen's 
        Handbook of marine craft hydrodynamics and motion control, Chapter 15.3.

        Args:
            m           The mass coefficient
            d           The damper coefficient
            k           The spring coefficient
            omega_b     The control bandwidth
            zeta        The relative damping ratio
        Returns:
            K_p         The proportional gain
            K_d         The derivative gain
            K_i         The integral gain
        """
        self.zeta = zeta
        self.omega_b = omega_b
        self.omega_n = omega_b/sqrt(1 - 2*zeta**2+sqrt(4*zeta**4 - 4*zeta**2 + 2))
        K_p = self.m*self.omega_n**2 - self.k
        K_d = 2*zeta*self.omega_n*self.m - self.d
        K_i = self.omega_n*K_p/10

        return K_p, K_d, K_i

