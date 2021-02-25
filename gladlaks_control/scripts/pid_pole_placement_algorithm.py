from math import sqrt

def pid_pole_placement_algorithm(m, d, k, omega_b, zeta):
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

    omega_n = omega_b/sqrt(1-2*zeta**2+sqrt(4*zeta**4-4*zeta**2+2))
    K_p = m*omega_n**2-k
    K_d = 2*zeta*omega_n*m-d
    K_i = omega_n*K_p/10

    return K_p, K_d, K_i