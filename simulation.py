import numpy as np
from jacobians import num_jacobian
from lqr_gain import dLQR
from dynamics import dynamics

def simulation(N, N2, dT, Q, R, C, D, x_0, xd, ud, Kd, x_d, u_opt, K_vt, x_sym, u_sym, sym_param, num_param):
    U, X = [], []
    x = x_0

    for k in range(N2):

    # Stabilizating the desired trajectory
        if k >= N:
            xd[:,k] = x_d[:,-1]
            # constant k 
            A_d, B_d = num_jacobian(x_sym, u_sym, sym_param, xd[:,k], 0, num_param, N, C, D, dT, 'cons')
            Kd[k] = dLQR(A_d, B_d, Q, Q, R, 'cons')
            u = -np.dot(Kd[k], x - xd[:,k]) 

        # Trajectory Tracking  
        if k < N:
            xd[:,k] = x_d[:,k]
            ud[k] = u_opt[k] 
            Kd[k] = K_vt[k]
            u = -np.dot(Kd[k], x - xd[:,k]) + ud[k]
        x = x + dynamics(x, u, num_param,'num')*dT
        U.append(u)
        X.append(x)

    return np.array(X), np.array(U)