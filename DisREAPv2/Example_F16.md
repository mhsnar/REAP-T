
 we use a simplified model to represent the lateral dynamics of a F-16 aircraft (see Fig. \ref{fig:3}), and implement the proposed MPC to control its roll and side-slip angles. The dynamics of the aircraft can be modelled as \cite{suresh2005nonlinear} $\dot{x}=$ $A_c x+B_c u$, where $x=\left[q~\alpha~\theta_r~\theta_s\right]^{\top}$ with $q$ being the roll rate, $\alpha$ being the yaw rate, $\theta_r$ being the roll angle, and $\theta_s$ being the side-slip angle, and $u=\left[\delta_a~\delta_r\right]^{\top}$ with $\delta_a$ and $\delta_r$ being aileron and rudder deflection commands, respectively. We use the matrices $A_c$ and $B_c$ given in \cite{suresh2005nonlinear}, and discretize the system with a sampling period of $100$ milliseconds. It is easy to show that, for any given desired roll and side-slip angles, the set of admissible steady-state configurations can be characterized by a two-dimensional vector (i.e., $\theta\in\mathbb{R}^2$). 


% $$
% \begin{gathered}
% A_c=\left[\begin{array}{cccc}
% -3.598 & 0.1968 & -35.180 & 0 \\
% -0.0377 & -0.3579 & 5.884 & 0 \\
% 0.0688 & -0.9957 & -0.2163 & 0.0733 \\
% 0.9947 & 0.1027 & 0 & 0
% \end{array}\right]\\
% B_c=\left[\begin{array}{cccc}
% 14.65 & 0.2179 & -0.0054 & 0 \\
% 6.538 & -3.087 & 0.0516 & 0
% \end{array}\right]^{\top} .
% \end{gathered}
% $$


%  The steady state and input are parameterized by \(\theta \in \mathbb{R}^2\) through the matrices
% \begin{align*}\label{eq:NumericalExample}
%     M_{1} &= \begin{bmatrix} 0.0123 &-0.1195 & 1 & 0 \\ -0.0075 & 0.0727 & 0 & 1 \end{bmatrix}^{\mathrm{T}}, \\
%     M_{2} &= \begin{bmatrix} 1.5019 & 2.0258 \\ 0.0009 & -0.0083 \end{bmatrix}^{\mathrm{T}},     N = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}.
% \end{align*}


% The desired roll and side-slip angles are 4.9 and 1.9 [deg], respectively. The system is subject to state constraints \cite{hosseinzadeh2022reference} $\left|\theta_r\right| \leq 5$ [deg] and $\left|\theta_s\right| \leq 2$ [deg], and input constraints $\left|\delta_a\right| \leq 10$ [deg] and $\left|\delta_r\right| \leq 15$ [deg]. We implement the proposed MPC with $N=4$, $Q_x=$ $\operatorname{diag}(0.1,0.1,10,10)$, $Q_r=\operatorname{diag}(3000,100)$, $Q_u=\operatorname{diag}(0.1,0.1)$. We would like to have minimum control effort at steady-state (i.e., $u_{des}=0$) with no preference on the steady state; thus, we utilize a quadratic $f(\theta)$ with $Q_{sx}=0$ and $Q_{su}=I_2$ (see Remark \ref{remark:Functionf}). Note that since $u_{des}=0$ does not correspond to an admissible steady-state configuration, the proposed steady-state aware MPC will steer the steady-state configuration of the aircraft to the best admissible one.  



% Fig. \ref{fig:4} presents the time profile of system states, control inputs, and the characterizing vector $\theta$. From this figure, the roll and side-slip angles converge to the desired values without violating the system constraints. Also, $\theta\rightarrow\theta^\diamond=[4.89~1.89]^\top$ (see Eq. \eqref{eq:Suboptimal}), and consequently ${\color{blue}\mathbf{u_s}}\rightarrow[7.36~9.91]^\top$ which is the best admissible steady input. 




% \begin{figure}[!t]
% 	\centering
%  \centering
% 	\includegraphics[width=\columnwidth,height=5cm]{F16.pdf}
% 	\caption{Lateral control of a F-16 aircraft.}
% 	\label{fig:3}
% \end{figure}



% \begin{figure}[!t]
% 	\centering
%  \centering
% 	\includegraphics[width=4.2cm]{EXP1.eps}
% 	\includegraphics[width=4.2cm]{EXP2.eps}\\	\includegraphics[width=4.2cm]{EXP3.eps}
%  \includegraphics[width=4.2cm]{EXP4.eps}\\
%  \includegraphics[width=4.2cm]{EXP5.eps}
%  \includegraphics[width=4.2cm]{EXP6.eps}\\
%   \includegraphics[width=4.2cm]{EXP7.eps}
%  \includegraphics[width=4.2cm]{EXP8.eps}\\
% 	\caption{Simulation results for lateral control with the proposed MPC.}
% 	\label{fig:4}
% \end{figure}
