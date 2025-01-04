
import numpy as np
    
def gradient_B_u(Bbar = None,errU = None,errX = None,errTX = None,QuBar = None,QxBar = None,NoS = None,hatLambda = None,Prediction_Horizion = None,AllConstraints = None,beta = None,Qn = None,BbarTx = None,BbarTu = None): 
    XConstraints1 = AllConstraints.x1
    XConstraints2 = AllConstraints.x2
    UConstraints1 = AllConstraints.u1
    UConstraints2 = AllConstraints.u2
    TXConstraints1 = AllConstraints.Tx1
    TXConstraints2 = AllConstraints.Tx2
    TUConstraints1 = AllConstraints.Tu1
    TUConstraints2 = AllConstraints.Tu2
    Ax1 = AllConstraints.Ax1
    Ax2 = AllConstraints.Ax2
    Cu1 = AllConstraints.Cu1
    Cu2 = AllConstraints.Cu2
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
    # Gradient with respect to u
    grad_B_u = 2 * QuBar * errU + 2 * np.transpose(Bbar) * QxBar * errX + 2 * np.transpose(Bbar(np.arange(NoS * Prediction_Horizion - (NoS - 1),NoS * Prediction_Horizion+1),:)) * Qn * errTX
    grad_B_u = grad_B_u - np.transpose(Bbar) * (np.multiply(np.multiply(hat_lambda_x1,(- beta)),Ax1) / (- beta * XConstraints1 + 1)) - np.transpose(Bbar) * (np.multiply(np.multiply(hat_lambda_x2,(- beta)),Ax2) / (- beta * XConstraints2 + 1)) - np.multiply(np.multiply(hat_lambda_u1,(- beta)),Cu1) / (- beta * UConstraints1 + 1) - np.multiply(np.multiply(hat_lambda_u2,(- beta)),Cu2) / (- beta * UConstraints2 + 1) - (np.transpose(BbarTx)) * (np.multiply(hat_lambda_omega_x1,(- beta)) / (- beta * TXConstraints1 + 1)) - (- np.transpose(BbarTx)) * (np.multiply(hat_lambda_omega_x2,(- beta)) / (- beta * TXConstraints2 + 1)) - (np.transpose(BbarTu)) * (np.multiply(hat_lambda_omega_u1,(- beta)) / (- beta * TUConstraints1 + 1)) - (- np.transpose(BbarTu)) * (np.multiply(hat_lambda_omega_u2,(- beta)) / (- beta * TUConstraints2 + 1))
    # TXConstraints1 = M1*theta-0.98*Xconstraint+1/beta;
# TXConstraints2 = -M1*theta-0.98*Xconstraint+1/beta;
# TUConstraints1 = M2*theta-0.98*Uconstraint+1/beta;
# TUConstraints2 =- M2*theta-0.98*Uconstraint+1/beta;
    return grad_B_u
    
    return grad_B_u