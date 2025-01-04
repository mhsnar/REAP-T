import numpy as np
    
def Phi(hatLambda = None,grad_B_lambda = None,hat_lambda = None): 
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
    # Adjust gradients based on conditions for Phi(s)
    lambda_ = np.array([[hat_lambda_x1],[hat_lambda_x2],[hat_lambda_u1],[hat_lambda_u2],[hat_lambda_omega_x1],[hat_lambda_omega_x2],[hat_lambda_omega_u1],[hat_lambda_omega_u2],[hat_lambda_omega_x1_eps],[hat_lambda_omega_x2_eps],[hat_lambda_omega_u1_eps],[hat_lambda_omega_u2_eps]])
    for k in np.arange(1,len(hat_lambda)+1).reshape(-1):
        if lambda_(k) > 0 or (lambda_(k) == 0 and grad_B_lambda(k) >= 0):
            phi[k] = 0
        else:
            if k == 126:
                sd = 1
            phi[k] = - (grad_B_lambda(k))
    
    phi = np.transpose(phi)
    return phi
    
    return phi