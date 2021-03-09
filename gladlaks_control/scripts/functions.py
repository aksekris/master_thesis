#!/usr/bin/env python
# Written by Aksel Kristoffersen

import numpy as np

def quaternion_to_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Args:
        Q (list) A 4 element array representing the quaternion in ROS 


    Return:
        rot_matrix (np.arryay) A 3x3 element matrix representing the full 3D rotation matrix. 
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

def control_forces_to_control_inputs(tau_c, u_max):
    input_matrix = [[-0.707,  0.000,  0.000,  0.707,  0.707,  0.000,  0.000, -0.707], 
                    [-0.707,  0.000,  0.000, -0.707,  0.707,  0.000,  0.000,  0.707],
                    [ 0.000,  1.000,  1.000,  0.000,  0.000,  1.000,  1.000,  0.000],
                    [ 0.046, -0.220, -0.220,  0.046, -0.046,  0.220,  0.220, -0.046],
                    [-0.046, -0.120,  0.120,  0.046,  0.046,  0.119, -0.119, -0.046],
                    [-0.324,  0.000,  0.000,  0.325, -0.325,  0.000,  0.000,  0.324]]
    input_matrix_arr = np.array(input_matrix)
    pseudo_inv_matrix = np.dot(input_matrix_arr.T, np.linalg.inv(np.dot(input_matrix_arr, input_matrix_arr.T)))
    control_inputs = np.dot(pseudo_inv_matrix, tau_c)
    
    for index, control_input in enumerate(control_inputs):
        if abs(control_input) > u_max[index]:
            gain = abs(control_input)/u_max[index]
            print(index)
            print(gain)
            control_inputs[index] = control_input/gain
            self_gain = abs(input_matrix[0][index])
            for index2, control_input2 in enumerate(control_inputs):
                if not index == index2:
                    target_gain = abs(input_matrix[0][index2])
                    try:
                        relative_gain = self_gain/target_gain
                        control_inputs[index2] = control_input2/gain*relative_gain
                    except:
                        continue
                        
    return control_inputs

def control_input_to_control_forces(control_input):
    input_matrix = [[-0.707,  0.000,  0.000,  0.707,  0.707,  0.000,  0.000, -0.707], 
     [-0.707,  0.000,  0.000, -0.707,  0.707,  0.000,  0.000,  0.707],
     [ 0.000,  1.000,  1.000,  0.000,  0.000,  1.000,  1.000,  0.000],
     [ 0.046, -0.220, -0.220,  0.046, -0.046,  0.220,  0.220, -0.046],
     [-0.046, -0.120,  0.120,  0.046,  0.046,  0.119, -0.119, -0.046],
     [-0.324,  0.000,  0.000,  0.325, -0.325,  0.000,  0.000,  0.324]]
    control_forces = np.dot(input_matrix, control_input)
    return control_forces

if __name__ == '__main__':
    tau_c = [5, 0, 0, 0, 0, 0]
    u_max = [1, 1, 1, 1, 1, 1, 1, 1]

    u = control_forces_to_control_inputs(tau_c, u_max)
    print('Control input:')
    print(u)
    tau_c = control_input_to_control_forces(u)
    print('Control forces:')
    print(tau_c)
