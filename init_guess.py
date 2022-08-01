
# The initial guess 
def init_guess(problem, N, u, q1, q2, dq1, dq2, x_0, x_f):
    for k in range(N): 
        tk = k/N
        problem.set_initial(q1[k], x_0[0]*(1-tk) + tk*x_f[0]) 
        problem.set_initial(q2[k], x_0[1]*(1-tk) + tk*x_f[1])
        problem.set_initial(dq1[k], x_0[2]*(1-tk) + tk*x_f[2])
        problem.set_initial(dq2[k], x_0[3]*(1-tk) + tk*x_f[3])
        problem.set_initial(u[k], 0)