![Parrot Drone](Drone%20(1).eps)

## Model Structure
Suppose that the yaw angle of the quadcopter is very small; see Subsection [Identification Procedure](#sec: IdentificationProcedure) for details on how one can regulate the yaw angle to zero. Taking inspiration from [Pinto et al., 2020](#), [Santos, 2019](#), we propose the following structure to describe the position dynamics of a quadcopter:

$$
\begin{aligned}
\ddot{p}_x + \alpha_x \dot{p}_x &= \beta_x u_x,\\
\ddot{p}_y + \alpha_y \dot{p}_y &= \beta_y u_y,\\
\ddot{p}_z + \alpha_z \dot{p}_z &= \beta_z u_z,
\end{aligned}
$$

where $ p_x, p_y, p_z \in \mathbb{R} $ are X, Y, and Z positions of the quadcopter in the global Cartesian coordinate system, \( \alpha_x, \alpha_y, \alpha_z, \beta_x, \beta_y, \beta_z \in \mathbb{R} \) are system parameters, and \( u_x, u_y, u_z \in \mathbb{R} \) are control inputs on X, Y, and Z directions.

We use the **Parrot Drone Support from MATLAB** to send control commands to the Parrot Bebop 2 drone. In this framework, \( u_x \) is the pitch angle (in [rad]), \( u_y \) is the roll angle (in [rad]), and \( u_z \) is the vertical velocity (in [m/s]); see Fig. [Parrot](#fig:Parrot).

The dynamical model can be expressed via the following state-space equations:

$$
\begin{aligned}
\dot{x} &=
\begin{bmatrix}
0 & 1 & 0 & 0 & 0 & 0 \\
0 & -\alpha_x & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & -\alpha_y & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & -\alpha_z
\end{bmatrix} x +
\begin{bmatrix}
0 & 0 & 0 \\
\beta_x & 0 & 0 \\
0 & 0 & 0 \\
0 & \beta_y & 0 \\
0 & 0 & 0 \\
0 & 0 & \beta_z
\end{bmatrix} u,\\

y &=
\begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0
\end{bmatrix} x,
\end{aligned}
$$

where \( x = \begin{bmatrix} p_x & \dot{p}_x & p_y & \dot{p}_y & p_z & \dot{p}_z \end{bmatrix}^\top \) and \( u = \begin{bmatrix} u_x & u_y & u_z \end{bmatrix}^\top \).

Note that the model is consistent with practical observations:

1. Dynamics along X, Y, and Z directions are decoupled, as one can control the position of the quadcopter along X, Y, and Z directions separately.
2. When \( \dot{p}_x \neq 0 \) (\( \dot{p}_y \neq 0 \) and \( \dot{p}_z \neq 0 \)) and \( u_x = 0 \) (\( u_y = 0 \), \( u_z = 0 \)), the quadcopter continues moving in the respective direction while its velocity decreases to zero. This observation suggests that \( \alpha_x, \alpha_y, \alpha_z \in \mathbb{R}_{>0} \).
3. When pitch and roll angles are small, the quadcopter behaves as a second-order system.
