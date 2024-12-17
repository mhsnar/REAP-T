# F-16 Aircraft Model

<img src="Pics/F16.png" alt="Welcome Image" style="width:75%;">

## Model Structure

 we use a simplified model to represent the lateral dynamics of a F-16 aircraft (see Fig. \ref{fig:3}), and implement the proposed MPC to control its roll and side-slip angles. The dynamics of the aircraft can be modelled as \cite{suresh2005nonlinear} $\dot{x}=$ $A_c x+B_c u$, where $x=\left[q~\alpha~\theta_r~\theta_s\right]^{\top}$ with $q$ being the roll rate, $\alpha$ being the yaw rate, $\theta_r$ being the roll angle, and $\theta_s$ being the side-slip angle, and $u=\left[\delta_a~\delta_r\right]^{\top}$ with $\delta_a$ and $\delta_r$ being aileron and rudder deflection commands, respectively. We use the matrices $A_c$ and $B_c$ given in \cite{suresh2005nonlinear}, and discretize the system with a sampling period of $100$ milliseconds. It is easy to show that, for any given desired roll and side-slip angles, the set of admissible steady-state configurations can be characterized by a two-dimensional vector (i.e., $\theta\in\mathbb{R}^2$). 


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
\end{array}\right]^{\top} .
\end{gathered}
$$

