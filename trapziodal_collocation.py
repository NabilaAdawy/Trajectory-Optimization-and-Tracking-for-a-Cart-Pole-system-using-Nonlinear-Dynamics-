from dynamics import dynamics

# collocation constraints
def trapz_colloc(problem, u, state, N, dt, num_param):
    for k in range(N-1):
        p1 = dynamics(state[:,k],u[:,k], num_param, 'casa')
        p2 = dynamics(state[:,k+1], u[:,k+1], num_param, 'casa')
        hk = dt 
        x_new = state[:,k] + hk*(p2+p1)/2 
        problem.subject_to(state[:,k+1]==x_new) 