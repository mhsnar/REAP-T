    
def ARMechanism(sigma_hat_u = None,sigma_hat_u0 = None,hat_u0 = None,hat_u = None,hatLambda0 = None,hatLambda = None): 
    if sigma_hat_u <= sigma_hat_u0:
        hatLambda = hatLambda
        hat_u = hat_u
    else:
        hatLambda = hatLambda0
        hat_u = hat_u0
    
    return hat_u,hatLambda
    
    return hat_u,hatLambda