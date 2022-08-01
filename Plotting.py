import numpy as np
from matplotlib.pyplot import *

def plotting(t1, t2, p, dist, theta, ang, U, u_opt):
    figure(figsize=(25, 8))
    plot(t2, p , linewidth=2.0, label='distance')
    plot(t1, dist[:-1],'r', linewidth=2.0, label='opt_distance')
    grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
    grid(True)
    xlabel(r'Time  $T$ (min)')
    legend()
    show()

    figure(figsize=(25, 8))
    plot(t2, theta, linewidth=2.0, label='angle')
    plot(t1, ang[:-1],'r', linewidth=2.0, label='opt_angle')
    grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
    grid(True)
    xlabel(r'Time  $T$ (min)')
    legend()
    show()

    figure(figsize=(25, 8))
    plot(t2, U, label='control')
    plot(t1, u_opt,'r', linewidth=2.0, label='opt_control')
    grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
    grid(True)
    ylabel(r'Control $\mathbf{u}[k]$')
    xlabel(r'Time  $T$ (min)')
    legend()
    show()

