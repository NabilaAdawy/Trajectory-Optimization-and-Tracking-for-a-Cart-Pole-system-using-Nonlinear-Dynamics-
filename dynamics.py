import sympy as sp
import numpy as np
from casadi import *
import casadi as ca
s, c = sp.sin, sp.cos

def dynamics(x,u,param, label):
    I, m, M, L, g = param
    if label == 'sym':
        theta, dtheta, p, dp = x
        # Nonlinear Symbolic Dynamics
        denom =  I*0 + M + m*(s(theta)**2) 
        ddp = (L*m*s(theta)*dtheta**2 + u + m*g*c(theta)*s(theta))/denom
        ddtheta = (-L*m*c(theta)*s(theta)*dtheta**2 - u*c(theta) -(M+m)*g*s(theta))/(L*denom)
        return dtheta, ddtheta, dp, ddp
        
    if label == 'casa':
        # Nonlinear Dynamics for Casadi
        dx1 = x[2]  
        dx2 = x[3]   
        denom = M + m*(ca.sin(x[1])**2) 
        ddx1 = (L*m*ca.sin(x[1])*x[3]**2 + u + m*g*ca.cos(x[1])*ca.sin(x[1]))/denom
        ddx2 = (-L*m*ca.cos(x[1])*ca.sin(x[1])*x[3]**2 - u*ca.cos(x[1]) -(M+m)*g*ca.sin(x[1]))/(L*denom)
        return vertcat(dx1,dx2,ddx1,ddx2)   
         
    elif label == 'num':
        theta, dtheta, p, dp = x
        # Nonlinear Numerical Dynamics
        denom =  I*0 + M + m*(1 - np.cos(theta)**2) 
        ddp = (L*m*np.sin(theta)*dtheta**2 + u + m*g*np.cos(theta)*np.sin(theta))/denom
        ddtheta = (-L*m*np.cos(theta)*np.sin(theta)*dtheta**2 - u*np.cos(theta) -(M+m)*g*np.sin(theta))/(L*denom)
        return np.array([dtheta, ddtheta, dp, ddp], dtype='double')