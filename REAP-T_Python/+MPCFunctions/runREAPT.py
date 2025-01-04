
import numpy as np
    
def runREAPT(A = None,B = None,C = None,D = None,Xconstraint = None,Uconstraint = None,x0 = None,r = None,Qx = None,Qu = None,Qv = None,DeltaT = None,Prediction_Horizon = None,Omegastar = None,n = None,checkbox1 = None,checkbox2 = None,checkbox3 = None,checkbox4 = None,ModeDropdown = None,AllConstraints = None): 
    # Initialize parameters and states
    Ad,Bd,Cd,Dd,NoS,NoI,NoO = Functions.initialize(A,B,C,D,DeltaT)
    # Compute MPC
    if str(ModeDropdown.Value) == str('Lyapunov-Based'):
        x,u_app,Sigmas = Functions.computeREAP_Lyapanov(Ad,Bd,Cd,Dd,Xconstraint,Uconstraint,x0,r,NoS,NoI,NoO,Qx,Qu,Qv,DeltaT,Prediction_Horizon,Omegastar,n)
    else:
        x,u_app,Sigmas = Functions.computeREAP_Omegastar(Ad,Bd,Cd,Dd,Xconstraint,Uconstraint,x0,r,NoS,NoI,NoO,Qx,Qu,Qv,DeltaT,Prediction_Horizon,Omegastar,n)
    
    # save('results.mat', 'x', 'u_app', 'Sigmas', 'AllConstraints');
    assignin('base','x',x)
    assignin('base','u_app',u_app)
    assignin('base','Sigmas',Sigmas)
    assignin('base','AllConstraints',AllConstraints)
    print('REAP ended!')
    print('Plotting the graphs has started!...')
    plotFlags = np.array([checkbox1.Value,checkbox2.Value,checkbox3.Value,checkbox4.Value])
    
    plott_EXP
    print('Plotting the graphs ended!')
    return x,u_app
    
    return x,u_app