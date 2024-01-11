import numpy as np

def dynamics(x, tau_RB):

    # Parameters
    m = 12
    l = 457e-3
    b = 338e-3
    h = 254e-3
    r_bg = np.array([0, 0, 0])
    r_bb = np.array([0, 0, -0.01])
    g = 9.81

    # State vector components
    pos = x[:3]
    Theta = x[3:6]
    vel = x[6:9]
    omega = x[9:12]
    nu = np.concatenate((vel, omega), axis=0)

    # Mass matrix
    I_xx = 1/12 * m * (h**2 + l**2)
    I_yy = 1/12 * m * (b**2 + h**2)
    I_zz = 1/12 * m * (b**2 + l**2)

    I = np.diag([I_xx, I_yy, I_zz])

    M_11 = m * np.eye(3)
    M_12 = -m * skew(r_bg)
    M_21 = m * skew(r_bg)
    M_22 = I

    M_RB = np.block([[M_11, M_12],
                    [M_21, M_22]])

    # Coriolis–Centripetal Matrix
    vel = np.zeros(3)  # velocity vector
    omega = np.zeros(3)  # angular velocity vector

    C_11 = np.zeros((3, 3))
    C_12 = -skew(M_11 @ vel + M_12 @ omega)
    C_21 = C_12
    C_22 = -skew(M_21 @ vel + M_22 @ omega)

    C_RB = np.block([[C_11, C_12],
                    [C_21, C_22]])

    # External Forces and Moments
    W = m * g
    B = W
    f_ng = np.array([0, 0, W])  # gravity in NED
    f_nb = -np.array([0, 0, B])  # buoyancy in NED

    # Rotation matrix
    phi = Theta[0]
    theta = Theta[1]
    psi = Theta[2]

    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)

    R = np.array([
        [c_psi*c_theta - s_psi*c_phi*s_theta, -s_psi*c_phi - c_psi*s_theta*s_phi, c_psi*s_theta*c_phi + s_psi*s_phi],
        [s_psi*c_theta + c_psi*s_phi*s_theta, c_psi*c_phi - s_phi*s_theta*s_psi,  s_psi*s_theta*c_phi - c_psi*s_phi],
        [-s_theta*c_phi,                      s_theta*s_phi,                      c_theta*c_phi]
    ])

    T = np.array([
        [1, s_phi * s_theta, c_phi * s_theta],
        [0, c_phi, -s_phi],
        [0, s_phi / c_theta, c_phi / c_theta]
    ])

    R_T = np.transpose(R)

    # Gravitation and boyancy
    f_g = R_T @ f_ng
    f_b = R_T @ f_nb

    g_eta = np.concatenate([f_g + f_b, np.cross(r_bg, f_g) + np.cross(r_bb, f_b)])

    # External viscous damping matrix
    D = 5 * np.eye(6)

    # Dynamic equations
    J = np.block([[R, np.zeros((3, 3))],
                [np.zeros((3, 3)), T]])

    eta_dot = J @ nu
    nu_dot = np.linalg.inv(M_RB) @ (tau_RB - C_RB @ nu - D @ nu - g_eta)

    x_dot = np.concatenate((eta_dot, nu_dot))

    return x_dot

def skew(vector):
    return np.array([[0, -vector[2], vector[1]],
                     [vector[2], 0, -vector[0]],
                     [-vector[1], vector[0], 0]])
