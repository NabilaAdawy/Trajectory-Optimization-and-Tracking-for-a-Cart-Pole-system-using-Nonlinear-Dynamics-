import numpy as np
from numpy import genfromtxt
from matplotlib.pyplot import *
from scipy.integrate import odeint
from scipy.interpolate import CubicSpline

# Defining the system dynamics


def M_term(q, parameters):
    I, m, M, L, g = parameters
    q1, q2 = q[0], q[1]

    d11 = M + m
    d12 = m * L * np.cos(q2)
    d21 = d12
    d22 = 0*I + m*L**2
    return np.array([[d11, d12], [d21, d22]], dtype='double')


def h_term(q, parameters):
    I, m, M, L, g = parameters
    q1, q2, dq1, dq2 = q[0], q[1], q[2], q[3]
    h1 = -m * L * np.sin(q2) * dq2**2
    h2 = m * g * L * np.sin(q2)
    return np.array([h1, h2], dtype='double')


def sysode(x, t, parameters, u):
    q, dq = x[:2], x[2:4]
    M_c = M_term(x, parameters)
    h_c = h_term(x, parameters)

    ddq = np.dot(np.linalg.inv(M_c),  u(t) - h_c)

    return np.concatenate([dq, ddq])


m1 = 0.3
M1 = 1
L1 = 0.5
g1 = 9.81
I1 = 0.006
params = I1, m1, M1, L1, g1

N = 50
NORM = np.zeros((0, N))
methods = ['imp', 'exp', 'var']

control = {}
trajectory = {}
solutions = {}

T = 2
dt = T/N
scale = 5

for method in methods:
    trajectory[method] = {}
    solutions[method] = {}
    # Reading the trajectory data
    ang = genfromtxt(
        fr'D:\IU\Fall22\TermProject\Trajectory-Optimization-Methods-for-Dynamical-Systems\Opt_traject_Data\angle_{method}_{N}N.csv', delimiter=',')
    dist = genfromtxt(
        fr'D:\IU\Fall22\TermProject\Trajectory-Optimization-Methods-for-Dynamical-Systems\Opt_traject_Data\distance_{method}_{N}N.csv', delimiter=',')
    trajectory[method]['ang'] = ang
    trajectory[method]['dist'] = dist
    trajectory[method]['q'] = np.array([dist[:-1], ang[:-1]])
    cont = genfromtxt(
        fr'D:\IU\Fall22\TermProject\Trajectory-Optimization-Methods-for-Dynamical-Systems\Opt_traject_Data\u_opt_{method}_{N}N.csv', delimiter=',')

    cont = np.array(cont)
    control[method] = cont

    # Integration parameters
    t0 = 0  # Initial time
    x0 = np.array([0, 0, 0, 0])  # Set initial state

    # _________________________ Interpolating control _________________________
    t_interpolate = np.linspace(t0, T, N)
    u_interpolate = CubicSpline(t_interpolate, cont)

    # ________________________ Open Loop Simulation _________________________
    t_new = np.linspace(t0, T, N*scale)
    sol = odeint(sysode, x0, t_new,
                 args=(params, u_interpolate, ))  # Integrate system

    sol = np.array([sol[i, :] for i in range(0, N*scale, scale)])
    x_pos, theta, dx, dtheta = np.split(sol, 4, axis=1)

    solutions[method]['ang'] = theta[:, 0]
    solutions[method]['dist'] = x_pos[:, 0]
    solutions[method]['q'] = np.array([x_pos[:, 0], theta[:, 0]])

    # calculating the integral error of position and angle
    position_intergal_error = np.trapz(abs(x_pos[:, 0] - dist[:-1]), dx=dt)
    angle_integral_error = np.trapz(abs(theta[:, 0] - ang[:-1]), dx=dt)
    print(
        f'the integral error for {method} position: ', position_intergal_error)
    print(f'the integral error for {method} Angle: ', angle_integral_error)


'''______________________________Plotting the norm of distance and angle_______________________________

'''
# calculating the integral error of norm
imp_norm = np.linalg.norm(
    solutions['imp']['q'] - trajectory['imp']['q'], axis=0)
exp_norm = np.linalg.norm(
    solutions['exp']['q'] - trajectory['exp']['q'], axis=0)
var_norm = np.linalg.norm(
    solutions['var']['q'] - trajectory['var']['q'], axis=0)

norm_imp_integral_error = np.trapz(imp_norm, dx=dt)
norm_exp_integral_error = np.trapz(exp_norm, dx=dt)
norm_var_integral_error = np.trapz(var_norm, dx=dt)
print('the integral error for norm_imp: ', norm_imp_integral_error)
print('the integral error for norm_exp: ', norm_exp_integral_error)
print('the integral error for norm_var: ', norm_var_integral_error)


# for k in range(N):
#     if imp_norm[k] > 0.2:
#         t_horizon = dt*k
#         print("time of error Divergence (Implicit): ", t_horizon)
#     if var_norm[k] > 0.2:
#         t_horizon = dt*k
#         print("time of error Divergence (Variational): ", t_horizon)

t = np.array(range(N+1))*dt

# Plotting the norm of X for each method
figure(figsize=(25, 8))
plot(t[:-1], imp_norm, linewidth=2.0, label='imp_norm')
plot(t[:-1], exp_norm, linewidth=2.0, linestyle='--', label='exp_norm')
plot(t[:-1], var_norm, linewidth=2.0,  label='var_norm')
grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
grid(True)
xlabel(r'Time  $T$ (min)')
legend()
show()
