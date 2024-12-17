# F-16 Aircraft Model

<img src="Pics/F16.png" alt="Welcome Image" style="width:55%;">

## Model Structure

 we use a simplified model to represent the lateral dynamics of a F-16 aircraft (see Figure), and implement the proposed MPC to control its roll and side-slip angles. The dynamics of the aircraft can be modelled as $\dot{x}=$ $A_c x+B_c u$, where $x=\left[q~\alpha~\theta_r~\theta_s\right]^{\top}$ with $q$ being the roll rate, $\alpha$ being the yaw rate, $\theta_r$ being the roll angle, and $\theta_s$ being the side-slip angle, and $u=\left[\delta_a~\delta_r\right]^{\top}$ with $\delta_a$ and $\delta_r$ being aileron and rudder deflection commands, respectively. We use the matrices $A_c$ and $B_c$ given in \cite{suresh2005nonlinear}, and discretize the system with a sampling period of $100$ milliseconds. It is easy to show that, for any given desired roll and side-slip angles, the set of admissible steady-state configurations can be characterized by a two-dimensional vector (i.e., $\theta\in\mathbb{R}^2$). 


$$
\begin{gathered}
A_c=\left[\begin{array}{cccc}
-3.598 & 0.1968 & -35.180 & 0 \\
-0.0377 & -0.3579 & 5.884 & 0 \\
0.0688 & -0.9957 & -0.2163 & 0.0733 \\
0.9947 & 0.1027 & 0 & 0
\end{array}\right]\\
B_c=\left[\begin{array}{cccc}
14.65 & 0.2179 & -0.0054 & 0 \\
6.538 & -3.087 & 0.0516 & 0
\end{array}\right]^{\top}
C_c=\left[\begin{array}{cccc}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 
\end{array}\right].
\end{gathered}
$$

## State and Control Input Constraints

The state constraints are given by:

- Upper Bound for State: $X.U.B. = [\infty, \infty, 5, 2]^\top$
- Lower Bound for State: $X.L.B. = -[\infty, \infty, 5, 2]^\top$

The control input constraints are:

- Upper Bound for Control Input: $U.U.B = [10, 15]^\top$
- Lower Bound for Control Input: $U.L.B. = [-10, -15]^\top$

These constraints apply for all time steps $t$.

## MPC Parameters

In this example, with a sampling period of $\Delta T = 0.1$ seconds (100 milliseconds), the following parameters are used:

- State weighting matrix:  


$$
\begin{gathered}
Q_x=\left[\begin{array}{cccc}
0.1 & 0 & 0 & 0 \\
0 & 0.1 & 0 & 0 \\
0 & 0 & 10 & 0 \\
0 & 0 & 0 & 10 
\end{array}\right]^{\top} .
\end{gathered}
$$

- Control input weighting matrix:  

$$
\begin{gathered}
Q_u=\left[\begin{array}{cc}
0.1 & 0 \\
0 & 0.1 
\end{array}\right]^{\top} .
\end{gathered}
$$

- Desired state vector $r = [4.9, 1.9]^\top$
