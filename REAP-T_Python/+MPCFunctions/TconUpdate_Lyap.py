import numpy as np
    
def TconUpdate_Lyap(hat_u = None,ds = None,Sigma = None,grad_B_u = None,Abar = None,Bbar = None,x0 = None,psi = None,Ad = None,Prediction_Horizion = None,xbar = None,ubar = None,NoS = None,NoI = None,Xconstraint = None,Uconstraint = None,Uconstraint_down = None,Xconstraint_down = None,beta = None,K = None): 
    hat_u0 = hat_u - ds * Sigma * grad_B_u
    x_prr = Abar * x0 + Bbar * hat_u0
    invpsi = inv(psi)
    err = x_prr(np.arange(Ad.shape[1-1] * Prediction_Horizion - (Ad.shape[1-1] - 1),Ad.shape[1-1] * Prediction_Horizion+1)) - xbar
    LyapanovFF = np.transpose((err)) * psi * (err)
    TX1 = []
    TX2 = []
    coeff = np.eye(NoS)
    beta = 50
    for incc in np.arange(1,NoS+1).reshape(-1):
        TX1 = np.array([[TX1],[LyapanovFF - ((xbar(incc) - Xconstraint(incc)) ** 2 / (coeff(incc,:) * invpsi * np.transpose(coeff(incc,:)))) + 1 / beta]])
        a1 = ((xbar(incc) - Xconstraint(incc)) ** 2 / (coeff(incc,:) * invpsi * np.transpose(coeff(incc,:))))
        a2 = ((- xbar(incc) + Xconstraint_down(incc)) ** 2 / (coeff(incc,:) * invpsi * np.transpose(coeff(incc,:))))
        TX2 = np.array([[TX2],[LyapanovFF - ((- xbar(incc) + Xconstraint_down(incc)) ** 2 / (coeff(incc,:) * invpsi * np.transpose(coeff(incc,:)))) + 1 / beta]])
    
    TU1 = []
    TU2 = []
    for incc in np.arange(1,NoI+1).reshape(-1):
        b1 = ((ubar(incc) - Uconstraint(incc)) ** 2 / (K(incc,:) * invpsi * np.transpose(K(incc,:))))
        b2 = ((- ubar(incc) + Uconstraint_down(incc)) ** 2 / (K(incc,:) * invpsi * np.transpose(K(incc,:))))
        TU1 = np.array([[TU1],[LyapanovFF - ((ubar(incc) - Uconstraint(incc)) ** 2 / (K(incc,:) * invpsi * np.transpose(K(incc,:)))) + 1 / beta]])
        TU2 = np.array([[TU2],[LyapanovFF - ((- ubar(incc) + Uconstraint_down(incc)) ** 2 / (K(incc,:) * invpsi * np.transpose(K(incc,:)))) + 1 / beta]])
    
    TCon = np.array([[TX1],[TX2],[TU1],[TU2]])
    return TCon
    
    return TCon