# Trajectory Optimization Methods for Dynamical Systems

Implementation of Variational Integrators in Trajectory Optimization Problem

Studying the Trajectory optimization technique (Direct Collocation) with three different representation of the dynamical system (the usual implicit form of dynamics, explicit form, and novel method called variational integrators). Comparing the effect of number of decision variables on the speed of the trajectory optimization solution convergence. We found that the novel approach yielded a much faster trajectory convergence.

We are using CasADi python (CasADi is an open-source framework for nonlinear optimization and algorithmic differentiation) to do our simulations. The solution is obtained using IPOPT (Interior Point Optimizer) solver. 


The novelty we seek is to compare between the three mentioned techniques, and prove that the number of the decision variables used in the trajectory optimization problem affects on the speed of the solution convergence. The less the number of decision variables the faster the solution converges. 

This could be useful for dynamical systems with larger DoF such as quadcopter or quadruped. 

We also combine all the processes, from calculating the optimal trajectory, to designing the appropriate controller to follow this trajectory and then stabilize the system at the desired conditions. 

Here you can see comparison of the time it take the trajectory to converge between the three mentioned techniques
![figure1](https://user-images.githubusercontent.com/22743949/203214160-c518ed3c-9a91-4ad3-bd25-770c5360ac2c.png)
![figure2](https://user-images.githubusercontent.com/22743949/203214165-de8ca39e-76b0-4679-b6b8-13825b3d7ec2.png)
