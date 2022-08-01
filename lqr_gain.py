import numpy as np
from scipy.linalg import solve_discrete_are as dare

# Time Vatiant LQR or contant LQR
def dLQR(A, B, H, Q, R, label):
    if label == 'tv':
        N = 200 
        n = 4
        P=H
        K=np.zeros((N,n))
        for i in range(1,N):
            # Compute the LQR gain
            gain = np.linalg.inv(R + B[N-i].T @ P @ B[N-i]) @ B[N-i].T @ P @ A[N-i]
            K[N - i, :] = gain
            # Update with difference Riccati equation
            P = Q + gain.T @ R @ gain + (A[N-i]- B[N-i] @ gain).T @ P @ (A[N-i]- B[N-i] @ gain)
        return K

    elif label == 'cons':
        # Solve the DARE
        S = dare(A, B, Q, R)
        R_inv = np.linalg.inv(R + B.T @ S @ B )
        K = R_inv @ (B.T @ S @ A)
        return K   