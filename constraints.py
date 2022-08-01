from casadi import *

# Defining control and State constraints.
def constraints(problem, u_max, u, d_max, q1, q2, dq1, dq2, x_0, x_f):
    # Bounds on control (motor saturation)
    problem.subject_to(problem.bounded(-u_max, u, u_max))

    # path constraints (keep cart on track)
    problem.subject_to(problem.bounded(-d_max, q1, d_max))

    # Initial condition
    problem.subject_to(q1[0]==x_0[0])  
    problem.subject_to(q2[0]==x_0[1])  
    problem.subject_to(dq1[0]==x_0[2])  
    problem.subject_to(dq2[0]==x_0[3])  

    # Final condition
    problem.subject_to(q1[-2]==x_f[0])  
    problem.subject_to(q2[-2]==x_f[1])  
    problem.subject_to(dq1[-2]==x_f[2])  
    problem.subject_to(dq2[-2]==x_f[3]) 

    problem.subject_to(q1[-1]==x_f[0])  
    problem.subject_to(q2[-1]==x_f[1])  
    problem.subject_to(dq1[-1]==x_f[2])  
    problem.subject_to(dq2[-1]==x_f[3]) 