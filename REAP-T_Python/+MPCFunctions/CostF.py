import numpy as np
    
def CostF(r = None,u_des = None,x0 = None,inc = None,Abar = None,Bbar = None,M1Bar = None,M2Bar = None,hat_u = None,QxBar = None,QuBar = None,Qn = None,Qv = None,Prediction_Horizion = None,NoS = None,DeltaT = None,M1 = None,M2 = None,N = None,u_app = None,theta = None): 
    xbar = M1 * theta
    ubar = M2 * theta
    V = N * theta
    ## real-world consideration of the state stimator
# if inc==1
#     x0=x(:,1);
# elseif inc>1
#     x0=[x(1,inc);(x(1,inc)-x(1,inc-1))/DeltaT;x(3,inc);(x(3,inc)-x(3,inc-1))/DeltaT;x(5,inc);(x(5,inc)-x(5,inc-1))/DeltaT;u_app(:,inc-1)];
# end
    x_pr = Abar * x0 + Bbar * hat_u
    errV = V - r
    ErrorV = (np.transpose(errV) * Qv * errV)
    norU = ubar - u_des
    NormU = (np.transpose(norU) * norU)
    errU = hat_u - M2Bar * theta
    ErrorU = np.transpose(errU) * QuBar * errU
    errX = x_pr - M1Bar * theta
    ErrorX = np.transpose(errX) * QxBar * errX
    errTX = x_pr(np.arange(NoS * Prediction_Horizion - (NoS - 1),NoS * Prediction_Horizion+1)) - xbar
    ErrorTX = np.transpose(errTX) * Qn * errTX
    sigma = ErrorV + ErrorU + ErrorX + ErrorTX + NormU
    return sigma
    
    return sigma