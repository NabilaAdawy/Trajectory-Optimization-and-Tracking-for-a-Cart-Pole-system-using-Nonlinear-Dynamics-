from sympy.utilities.lambdify import lambdify
import numpy as np
from dynamics import dynamics
import sympy as sp
from scipy.signal import cont2discrete

A_num, B_num = [], []

def jacobians(x_sym, u_sym, sym_param):
    I, m, M, L, g = sym_param
    # computing the jacobians symbolically
    f_sym = sp.Matrix(dynamics(x_sym, u_sym, sym_param,'sym'))
    Jx_sym = f_sym.jacobian([x_sym])
    Ju_sym = f_sym.jacobian([u_sym])
    
    # Convert the symbolic jacobians to numerical functions
    A_func = lambdify([x_sym, u_sym, [I, m, M, L, g]], Jx_sym)
    B_func = lambdify([x_sym, u_sym, [I, m, M, L, g]], Ju_sym)
    return A_func, B_func

# Evaluating the jacobians for N points of the desired trajectory or at specific point
def num_jacobian(x_sym, u_sym, sym_param, x_d, u_opt, num_param,N, C, D, dT, label):
    
    I1, m1, M1, L1, g1 = num_param
    A_func, B_func = jacobians(x_sym, u_sym, sym_param)

    if label == 'var':
        for k in range(N):
            A = A_func(x_d[:,k], u_opt[k], [I1, m1, M1, L1, g1])
            B = B_func(x_d[:,k], u_opt[k], [I1, m1, M1, L1, g1])
            # Discretize A and B
            A_d, B_d, C_d, D_d, _ = cont2discrete((A,B,C,D), dT)  
            A_num.append(A_d)
            B_num.append(B_d)   
        return A_num, B_num 

    elif label == 'cons':
        A = A_func(x_d, u_opt, [I1, m1, M1, L1, g1])
        B = B_func(x_d, u_opt, [I1, m1, M1, L1, g1])
        # Discretize A and B
        A_d, B_d, C_d, D_d, _ = cont2discrete((A,B,C,D), dT)  
        return A_d, B_d
