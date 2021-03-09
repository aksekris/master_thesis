#!/usr/bin/env python
# Written by Aksel Kristoffersen

import numpy as np
import quadprog

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
    input_matrix = [[-0.707,  0.000,  0.000,  0.707,  0.707,  0.000,  0.000, -0.707], 
                        [-0.707,  0.000,  0.000, -0.707,  0.707,  0.000,  0.000,  0.707],
                        [ 0.000,  1.000,  1.000,  0.000,  0.000,  1.000,  1.000,  0.000],
                        [ 0.046, -0.220, -0.220,  0.046, -0.046,  0.220,  0.220, -0.046],
                        [-0.046, -0.120,  0.120,  0.046,  0.046,  0.119, -0.119, -0.046],
                        [-0.324,  0.000,  0.000,  0.325, -0.325,  0.000,  0.000,  0.324]]
    input_matrix_arr = np.array(input_matrix)
    pseudo_inv_matrix = np.dot(input_matrix_arr.T, np.linalg.inv(np.dot(input_matrix_arr, input_matrix_arr.T)))
    pseudo_inv_matrix_neg = np.negative(pseudo_inv_matrix)

    # Find u_unb
    tau_unb = [10, 0, 0, 0, 1, 1]
    print('tau_unb:')
    print(tau_unb)
    u_unb = np.dot(pseudo_inv_matrix, tau_unb)
    print('u_unb:')
    print(u_unb)

    # Actuator limits
    u_max = np.array([1, 1, 1, 1, 1, 1, 1, 1])
    u_min = np.array([-1, -1, -1, -1, -1, -1, -1, -1])

    # Weights
    W = np.diag([0.01, 0.01, 0.01, 1, 1, 1])

    P = np.block([[W*1.000001, -W], [-W, W]]) # Cheat to make P pos def
    q = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    G = np.block([[pseudo_inv_matrix, -pseudo_inv_matrix], [-pseudo_inv_matrix, pseudo_inv_matrix]])
    h = np.block([u_max - u_unb, -u_min + u_unb])

    res = quadprog_solve_qp(P, q, G, h)
    tau_b = tau_unb + res[0:6] - res[6:12]
    print('tau_b:')
    print(tau_b)
    u_b = np.dot(pseudo_inv_matrix, tau_b)
    print('u_b')
    print(u_b)