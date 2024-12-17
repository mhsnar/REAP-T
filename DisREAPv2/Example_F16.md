# Quadcopter Model

![Parrot Drone](Drone%20(1).eps)

## Model Structure

We propose the following structure to describe the position dynamics of a quadcopter:

$$
\begin{aligned}
\dot{x}=A_c x+B_c u\\
\end{aligned}
$$

where $x=\begin{bmatrix}p_x & \dot{p}_x & p_y & \dot{p}_y & p_z & \dot{p}_z \end{bmatrix}^\top$

and $u=[u_x u_y u_z]$. 

With $p_x,p_y,p_z \in \mathbb{R}$ are $X$, $Y$, and $Z$ positions of the quadcopter in the global Cartesian coordinate system, $\alpha_x, \alpha_y, \alpha_z, \beta_x, \beta_y, \beta_z \in \mathbb{R}$ are system parameters, and $u_x, u_y, u_z \in \mathbb{R}$ are control inputs on $X$, $Y$, and $Z$ directions. In this framework:
- $u_x$ is the pitch angle (in radians),
- $u_y$ is the roll angle (in radians), and
- $u_z$ is the vertical velocity (in meters/second).

See Figure [Parrot](#parrot).


The dynamical model can be expressed in state-space form with the following matrices:

```math
A_c = 
\begin{bmatrix}
0 & 1.0000 & 0 & 0 & 0 & 0 \\
0 & -0.0527 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 1.0000 & 0 & 0 \\
0 & 0 & 0 & -0.0187 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1.0000 \\
0 & 0 & 0 & 0 & 0 & -1.7873
\end{bmatrix},
B_c =
\begin{bmatrix}
0 & 0 & 0 \\
-5.4779 & 0 & 0 \\
0 & 0 & 0 \\
0 & -7.0608 & 0 \\
0 & 0 & 0 \\
0 & 0 & -1.7382
\end{bmatrix}, C_c = I_6

