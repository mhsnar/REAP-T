import numpy as np
    
def Warmstarting(hatLambda = None,hat_u = None,NoS = None,NoI = None,Prediction_Horizion = None,M1 = None,M2 = None,x_pr = None,Omegastar = None,theta = None,K = None): 
    #Warm Starting
    hat_u_0 = []
    hat_lambda_x1_0 = []
    hat_lambda_x2_0 = []
    hat_lambda_u1_0 = []
    hat_lambda_u2_0 = []
    hat_lambda_omega_x1_0 = []
    hat_lambda_omega_x2_0 = []
    hat_lambda_omega_u1_0 = []
    hat_lambda_omega_u2_0 = []
    # hat_lambda_omega_x1_eps_0=[];
# hat_lambda_omega_x2_eps_0=[];
# hat_lambda_omega_u1_eps_0=[];
# hat_lambda_omega_u2_eps_0=[];
    
    hat_lambda_x1 = hatLambda.x1
    hat_lambda_x2 = hatLambda.x2
    hat_lambda_u1 = hatLambda.u1
    hat_lambda_u2 = hatLambda.u2
    hat_lambda_omega_x1 = hatLambda.omega_x1
    hat_lambda_omega_x2 = hatLambda.omega_x2
    hat_lambda_omega_u1 = hatLambda.omega_u1
    hat_lambda_omega_u2 = hatLambda.omega_u2
    hat_lambda_omega_x1_eps = hatLambda.omega_x1_eps
    hat_lambda_omega_x2_eps = hatLambda.omega_x2_eps
    hat_lambda_omega_u1_eps = hatLambda.omega_u1_eps
    hat_lambda_omega_u2_eps = hatLambda.omega_u2_eps
    for p in np.arange(1,Prediction_Horizion - 1+1).reshape(-1):
        hat_u_0 = np.array([[hat_u_0],[hat_u(np.arange(NoI * p + 1,NoI * p + NoI+1))]])
        hat_lambda_x1_0 = np.array([[hat_lambda_x1_0],[hat_lambda_x1(np.arange(NoS * p + 1,NoS * p + NoS+1))]])
        hat_lambda_x2_0 = np.array([[hat_lambda_x2_0],[hat_lambda_x2(np.arange(NoS * p + 1,NoS * p + NoS+1))]])
        hat_lambda_u1_0 = np.array([[hat_lambda_u1_0],[hat_lambda_u1(np.arange(NoI * p + 1,NoI * p + NoI+1))]])
        hat_lambda_u2_0 = np.array([[hat_lambda_u2_0],[hat_lambda_u2(np.arange(NoI * p + 1,NoI * p + NoI+1))]])
    
    for p in np.arange(1,Omegastar - 1+1).reshape(-1):
        hat_lambda_omega_x1_0 = np.array([[hat_lambda_omega_x1_0],[hat_lambda_omega_x1(np.arange(NoS * p + 1,NoS * (p + 1)+1))]])
        hat_lambda_omega_x2_0 = np.array([[hat_lambda_omega_x2_0],[hat_lambda_omega_x2(np.arange(NoS * p + 1,NoS * (p + 1)+1))]])
        hat_lambda_omega_u1_0 = np.array([[hat_lambda_omega_u1_0],[hat_lambda_omega_u1(np.arange(NoI * p + 1,NoI * (p + 1)+1))]])
        hat_lambda_omega_u2_0 = np.array([[hat_lambda_omega_u2_0],[hat_lambda_omega_u2(np.arange(NoI * p + 1,NoI * (p + 1)+1))]])
    
    #the same
    
    hat_lambda_omega_x1_eps_0 = hat_lambda_omega_x1_eps
    hat_lambda_omega_x2_eps_0 = hat_lambda_omega_x2_eps
    hat_lambda_omega_u1_eps_0 = hat_lambda_omega_u1_eps
    hat_lambda_omega_u2_eps_0 = hat_lambda_omega_u2_eps
    hat_u_0 = np.array([[hat_u_0],[M2 * theta + K * (x_pr(np.arange(NoS * Prediction_Horizion - NoS + 1,NoS * Prediction_Horizion+1)) - M1 * theta)]])
    hat_lambda_x1_0 = np.array([[hat_lambda_x1_0],[hat_lambda_x1(np.arange(NoS * Prediction_Horizion - NoS + 1,NoS * Prediction_Horizion+1))]])
    hat_lambda_x2_0 = np.array([[hat_lambda_x2_0],[hat_lambda_x2(np.arange(NoS * Prediction_Horizion - NoS + 1,NoS * Prediction_Horizion+1))]])
    hat_lambda_u1_0 = np.array([[hat_lambda_u1_0],[hat_lambda_u1(np.arange(NoI * Prediction_Horizion - NoI + 1,NoI * Prediction_Horizion+1))]])
    hat_lambda_u2_0 = np.array([[hat_lambda_u2_0],[hat_lambda_u2(np.arange(NoI * Prediction_Horizion - NoI + 1,NoI * Prediction_Horizion+1))]])
    hat_lambda_omega_x1_0 = np.array([[hat_lambda_omega_x1_0],[hat_lambda_omega_x1(np.arange(NoS * Omegastar - NoS + 1,NoS * Omegastar+1))]])
    hat_lambda_omega_x2_0 = np.array([[hat_lambda_omega_x2_0],[hat_lambda_omega_x2(np.arange(NoS * Omegastar - NoS + 1,NoS * Omegastar+1))]])
    hat_lambda_omega_u1_0 = np.array([[hat_lambda_omega_u1_0],[hat_lambda_omega_u1(np.arange(NoI * Omegastar - NoI + 1,NoI * Omegastar+1))]])
    hat_lambda_omega_u2_0 = np.array([[hat_lambda_omega_u2_0],[hat_lambda_omega_u2(np.arange(NoI * Omegastar - NoI + 1,NoI * Omegastar+1))]])
    hatLambda.x1 = hat_lambda_x1_0
    hatLambda.x2 = hat_lambda_x2_0
    hatLambda.u1 = hat_lambda_u1_0
    hatLambda.u2 = hat_lambda_u2_0
    hatLambda.omega_x1 = hat_lambda_omega_x1_0
    hatLambda.omega_x2 = hat_lambda_omega_x2_0
    hatLambda.omega_u1 = hat_lambda_omega_u1_0
    hatLambda.omega_u2 = hat_lambda_omega_u2_0
    hatLambda.omega_x1_eps = hat_lambda_omega_x1_eps_0
    hatLambda.omega_x2_eps = hat_lambda_omega_x2_eps_0
    hatLambda.omega_u1_eps = hat_lambda_omega_u1_eps_0
    hatLambda.omega_u2_eps = hat_lambda_omega_u2_eps_0
    return hatLambda,hat_u_0
    
    return hatLambda,hat_u_0