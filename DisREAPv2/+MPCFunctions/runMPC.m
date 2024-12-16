
function [x, u_app] = runMPC(A, B, C, D,Xconstraint,Uconstraint, x0,r,Qx, Qu, Qv, DeltaT, Prediction_Horizon, Omegastar, n,checkbox1, checkbox2, checkbox3,checkbox4,ModeDropdown,AllConstraints)

% Main function to run the MPC

% Validate inputs

% Initialize parameters and states
[Ad, Bd, Cd,Dd, NoS, NoI, NoO] = Functions.initialize(A, B, C, D, DeltaT);

% Compute MPC
if strcmp(ModeDropdown.Value, 'Lyapunov-Based') 

[x, u_app,Sigmas] = Functions.computeMPCLyapanov(Ad, Bd, Cd,Dd,Xconstraint,Uconstraint,x0,r, NoS, NoI, NoO, Qx, Qu, Qv, DeltaT, Prediction_Horizon, Omegastar, n);

else
[x, u_app,Sigmas] = Functions.computeMPC_Omegastar(Ad, Bd, Cd,Dd,Xconstraint,Uconstraint,x0,r, NoS, NoI, NoO, Qx, Qu, Qv, DeltaT, Prediction_Horizon, Omegastar, n);

end
% save('results.mat', 'x', 'u_app', 'Sigmas', 'AllConstraints');
assignin('base', 'x', x);
assignin('base', 'u_app', u_app);
assignin('base', 'Sigmas', Sigmas);
assignin('base', 'AllConstraints', AllConstraints);
disp('REAP ended!');
disp('Plotting the graphs has started!...');
plotFlags = [checkbox1.Value, checkbox2.Value, checkbox3.Value,checkbox4.Value]; % Logical array for plot states
plott_EXP
disp('Plotting the graphs ended!');
end
