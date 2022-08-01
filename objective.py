

def objective_func(problem, u, state, N, dt, Q, R, x_f):
    cost=0
    for k in range(N-1):
        cost += R*dt*((u[k].T @ u[k]) + (u[k+1].T @ u[k+1]))/2
        cost += Q*dt*((state[:,k] - x_f).T @ (state[:,k] - x_f))

    problem.minimize(cost) 