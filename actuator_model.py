#!/usr/bin/env python
# Written by Aksel Kristoffersen
# Documentation can be found in my master's thesis, chapter __ and __

import numpy as np

class ActuatorModel:
    def __init__(self, input_matrix, rotor_time_constant, u_max, u_min):
        self.input_matrix = input_matrix
        self.rotor_time_constant = rotor_time_constant
        self.u_max = np.array(u_max)
        self.u_min = np.array(u_min)
        self.pseudo_inv_input_matrix = moore_penrose_pseudo_inverse(input_matrix)

def moore_penrose_pseudo_inverse(matrix):
    return np.dot(np.array(matrix).T, np.linalg.inv(np.dot(np.array(matrix), np.array(matrix).T)))

