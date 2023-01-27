import numpy as np
import sympy as sp
from casadi import *
from lqr_gain import dLQR
from jacobians import num_jacobian
from plotting import plotting
from constraints import constraints
from objective import objective_func
from trapziodal_collocation import trapz_colloc
from init_guess import init_guess
from simulation import simulation

"""
Trajectory Optimization Using Casadi Framework.
"""

# Numerical parameters
I1, m1, M1, L1, g1 = 0.006, 0.3, 1, 0.5, 9.81
num_param = I1, m1, M1, L1, g1
d_max = 2  # track half length
d = 1  # desired cart distance
u_max = 25  # motor peak force
x_0 = [0, 0, 0, 0]  # Initial state
x_f = [d, np.pi, 0, 0]  # final state
Q_casa, R_casa = 1, 40  # gains for the objective function

# Simulation Parameters
N = 200  # opt trajectory points
T = 2  # total time
dT = T/N
t1 = np.linspace(0, T, N)

# Define NLP problem
problem = Opti()

# Decision variables
state = problem.variable(4, N+1)  # state of the system
q1 = state[0, :]  # cart distance
q2 = state[1, :]  # pole angle
dq1 = state[2, :]  # cart horizontal velocity
dq2 = state[3, :]  # pole angular velocity
u = problem.variable(1, N)   # control variables (motor force)

# Problem constraints
constraints(problem, u_max, u, d_max, q1, q2, dq1, dq2, x_0, x_f)

# objective function
objective_func(problem, u, state, N, dT, Q_casa, R_casa, x_f)

# Trapziodal collocation constraints
trapz_colloc(problem, u, state, N, dT, num_param)

# Initial Guess
init_guess(problem, N, u, q1, q2, dq1, dq2, x_0, x_f)

# solve NLP
problem.solver("ipopt")  # set numerical backend
sol = problem.solve()   # solve defined problem

# parsing the solution from sol
dist = sol.value(q1)
ang = sol.value(q2)
vel = sol.value(dq1)
ang_vel = sol.value(dq2)
u_opt = sol.value(u)
x_d = np.array([ang[:-1], ang_vel[:-1], dist[:-1], vel[:-1]])
u_opt = np.array(u_opt)

'''
Simulating cart-pole system while following the optimal trajectory 
with LQR controller, Then stabilizing it in the vertical up position.
'''

# System Symbols paramteres
I, m, M, L, g = sp.symbols(r'I m M L g')
sym_param = I, m, M, L, g
x_sym = sp.symbols(r'\theta \dot{\theta} x \dot{x}')
u_sym = sp.symbols(r'u')

C = np.array([[1, 0, 0, 0]])
D = np.array([[0, 0]])
Q = np.diag([12.5, 12.5, 12.5, 12.5])
R = np.diag([0.95])

# Evaluating the numerical jacobians at N points of the desired trajectory
A_num, B_num = num_jacobian(x_sym, u_sym, sym_param,
                            x_d, u_opt, num_param, N, C, D, dT, 'var')

# Compute the LQR gain K
K_vt = dLQR(A_num, B_num, Q, Q, R, 'tv')

# For Stabilization
N2 = 1000
T2 = 10
t2 = np.linspace(0, T2, N2)

xd = np.zeros((4, N2))
ud = np.zeros(N2)
Kd = np.zeros((N2, 4))

# Simulating the system
X, U = simulation(N, N2, dT, Q, R, C, D, x_0, xd, ud, Kd, x_d,
                  u_opt, K_vt, x_sym, u_sym, sym_param, num_param)
theta, dtheta, pos, dpos = X[:, 0], X[:, 1], X[:, 2], X[:, 3]

# Plotting the optimal and actual trajectories
plotting(t1, t2, pos, dist, theta, ang, U, u_opt)
