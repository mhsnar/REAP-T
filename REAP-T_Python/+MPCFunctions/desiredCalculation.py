
import numpy as np
    
def desiredCalculation(Ad = None,Bd = None,Cd = None,Dd = None,NoS = None,NoI = None,r = None): 
    X = np.array([[Ad - np.eye(len(Ad)),Bd,np.zeros((len(Ad),Cd.shape[1-1]))],[Cd,Dd,- np.eye(Cd.shape[1-1],Cd.shape[1-1])]])
    MN = null(X,'rational')
    M1 = MN(np.arange(1,NoS+1),:)
    N = MN(np.arange(1 + NoS + NoI,end()+1),:)
    Theta = pinv(N) * r
    M2 = MN(np.arange(NoS + 1,NoS + NoI+1),:)
    ubar = M2 * Theta
    xbar = M1 * Theta
    return xbar
    
    return xbar