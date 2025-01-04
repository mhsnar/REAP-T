import numpy as np
    
def computeREAP_Omegastar(Ad = None,Bd = None,Cd = None,Dd = None,Xconstraints = None,Uconstraints = None,x0 = None,Xbar = None,NoS = None,NoI = None,NoO = None,Qx = None,Qu = None,Qv = None,DeltaT = None,Prediction_Horizion = None,Omegastar = None,n = None,C = None): 
    # Compute MPC control action
# Implement the MPC logic here based on the provided code
    print('REAP has started!...')
    # Initialize variables
    
    u_des = np.zeros((NoI,1))
    # Initial conditions and reference
    
    x[:,1] = np.array([[x0],[np.zeros((NoI,1))]])
    r = Cd(:,np.arange(1,NoS - NoI+1)) * Xbar
    # State and control constraints
    Xconstraint_down = Xconstraints(:,1)
    Xconstraint = Xconstraints(:,end())
    Uconstraint_down = Uconstraints(:,1)
    Uconstraint = Uconstraints(:,end())
    Xconstraint = np.array([[Xconstraint],[1000 * np.ones((NoI,1))]])
    Xconstraint_down = np.array([[Xconstraint_down],[- 1000 * np.ones((NoI,1))]])
    # Extended matrices for prediction horizon
    QxBar = blkdiag(kron(np.eye(Prediction_Horizion),Qx))
    QuBar = blkdiag(kron(np.eye(Prediction_Horizion),Qu))
    Qx = np.array([[Qx,np.zeros((Qx.shape[1-1],Qu.shape[1-1]))],[np.zeros((Qu.shape[1-1],Qx.shape[1-1])),Qu]])
    Qn,K,__,__ = idare(Ad,Bd,Qx,Qu,[],[])
    K = - K
    psi = dlyap(np.transpose((Ad + Bd * K)),np.eye(NoS))
    # Placeholder for control input
    u_app = np.zeros((NoI,n))
    X = np.array([[Ad - np.eye(len(Ad)),Bd,np.zeros((len(Ad),Cd.shape[1-1]))],[Cd,Dd,- np.eye(Cd.shape[1-1],Cd.shape[1-1])]])
    MN = null(X,'rational')
    M1 = MN(np.arange(1,NoS - NoI+1),:)
    M2 = MN(np.arange(1 + NoS,+ NoS + NoI+1),:)
    M1 = np.array([[M1],[M2]])
    N = MN(np.arange(1 + NoS + NoI,end()+1),:)
    # Calculating x_pr:
    Abar = Ad
    for i in np.arange(2,Prediction_Horizion+1).reshape(-1):
        Abar = np.array([[Abar],[Ad ** (i)]])
    
    o,m = Bd.shape
    Bbar = np.zeros((o * Prediction_Horizion,m * Prediction_Horizion))
    for i in np.arange(1,Prediction_Horizion+1).reshape(-1):
        for j in np.arange(1,i+1).reshape(-1):
            A_power = Ad ** (i - j)
            # Element-wise multiplication using matrix multiplication
            block = A_power * Bd
            # Calculate the indices for insertion
            row_indices = np.arange((i - 1) * o + 1,i * o+1)
            col_indices = np.arange((j - 1) * m + 1,j * m+1)
            # Insert the block into the appropriate position in Bbar
            Bbar[row_indices,col_indices] = block
    
    beta = 100
    x_up = []
    u_up = []
    u_down = []
    x_down = []
    for i in np.arange(1,Prediction_Horizion+1).reshape(-1):
        u_up = np.array([[u_up],[Uconstraint]])
        u_down = np.array([[u_down],[Uconstraint_down]])
        x_up = np.array([[x_up],[Xconstraint]])
        x_down = np.array([[x_down],[Xconstraint_down]])
    
    # u_down=-u_up;
# x_down=-x_up;
    
    # Create QuBar using blkdiag
    QuBar = blkdiag(kron(np.eye(Prediction_Horizion),Qu))
    QxBar = blkdiag(kron(np.eye(Prediction_Horizion),Qx))
    M1Bar = M1
    M2Bar = M2
    for i in np.arange(1,Prediction_Horizion - 1+1).reshape(-1):
        M1Bar = np.array([[M1Bar],[M1]])
        M2Bar = np.array([[M2Bar],[M2]])
    
    AbarTx = (Ad + Bd * K)
    for i in np.arange(2,Omegastar+1).reshape(-1):
        AbarTx = np.array([[AbarTx],[(Ad + Bd * K) ** (i)]])
    
    BbarTx = AbarTx * Bbar(np.arange(NoS * Prediction_Horizion - (NoS - 1),NoS * Prediction_Horizion+1),:)
    AbarTx = AbarTx * Abar(np.arange(NoS * Prediction_Horizion - (NoS - 1),NoS * Prediction_Horizion+1),:)
    
    AbarTu = K
    for i in np.arange(1,Omegastar - 1+1).reshape(-1):
        AbarTu = np.array([[AbarTu],[K * ((Ad + Bd * K)) ** (i)]])
    
    BbarTu = AbarTu * Bbar(np.arange(NoS * Prediction_Horizion - (NoS - 1),NoS * Prediction_Horizion+1),:)
    AbarTu = AbarTu * Abar(np.arange(NoS * Prediction_Horizion - (NoS - 1),NoS * Prediction_Horizion+1),:)
    
    Number_of_Constraints = (Prediction_Horizion + Omegastar + 1) * (2 * Xconstraint.shape[1-1] + 2 * Uconstraint.shape[1-1])
    hat_lambda = np.zeros((Number_of_Constraints,1))
    hat_lambda_x1 = np.zeros((Xconstraint.shape[1-1] * (Prediction_Horizion),1))
    hat_lambda_x2 = np.zeros((Xconstraint.shape[1-1] * Prediction_Horizion,1))
    hat_lambda_u1 = np.zeros((Uconstraint.shape[1-1] * Prediction_Horizion,1))
    hat_lambda_u2 = np.zeros((Uconstraint.shape[1-1] * Prediction_Horizion,1))
    hat_lambda_omega_x1 = np.zeros((Xconstraint.shape[1-1] * (Omegastar),1))
    hat_lambda_omega_x2 = np.zeros((Xconstraint.shape[1-1] * (Omegastar),1))
    hat_lambda_omega_u1 = np.zeros((Uconstraint.shape[1-1] * (Omegastar),1))
    hat_lambda_omega_u2 = np.zeros((Uconstraint.shape[1-1] * (Omegastar),1))
    hat_lambda_omega_x1_eps = np.zeros((Xconstraint.shape[1-1],1))
    hat_lambda_omega_x2_eps = np.zeros((Xconstraint.shape[1-1],1))
    hat_lambda_omega_u1_eps = np.zeros((Uconstraint.shape[1-1],1))
    hat_lambda_omega_u2_eps = np.zeros((Uconstraint.shape[1-1],1))
    hat_u = np.zeros((Prediction_Horizion * Uconstraint.shape[1-1],1))
    hatLambda.x1 = hat_lambda_x1
    hatLambda.x2 = hat_lambda_x2
    hatLambda.u1 = hat_lambda_u1
    hatLambda.u2 = hat_lambda_u2
    hatLambda.omega_x1 = hat_lambda_omega_x1
    hatLambda.omega_x2 = hat_lambda_omega_x2
    hatLambda.omega_u1 = hat_lambda_omega_u1
    hatLambda.omega_u2 = hat_lambda_omega_u2
    hatLambda.omega_x1_eps = hat_lambda_omega_x1_eps
    hatLambda.omega_x2_eps = hat_lambda_omega_x2_eps
    hatLambda.omega_u1_eps = hat_lambda_omega_u1_eps
    hatLambda.omega_u2_eps = hat_lambda_omega_u2_eps
    ##
    flag = 0
    f = 1
    eta = 0.001
    sign = []
    Sigmas = []
    Sigmass = []
    sigma_values = []
    Total_Num_REAP = np.zeros((n,1))
    for inc in np.arange(1,n+1).reshape(-1):
        theta = pinv(MN(np.arange(1,NoS - NoI+1),:)) * Xbar
        xbar = M1 * theta
        ubar = M2 * theta
        V = N * theta
        if inc == 1:
            x0 = x(:,inc)
        else:
            if inc > 1:
                # Real-world consideration for state stimation
# x0=[x(1,inc);(x(1,inc)-x(1,inc-1))/DeltaT;x(3,inc);(x(3,inc)-x(3,inc-1))/DeltaT;x(5,inc);(x(5,inc)-x(5,inc-1))/DeltaT;u_app(:,inc-1)];
                x0 = x(:,inc)
        ## Terminal Conditions
        if inc == 1:
            yalmip('clear')
            # theta = sdpvar(3,1,'full');
# theta=[r(1) r(3) r(5)]';
            hat_u = sdpvar(NoI * Prediction_Horizion,1,'full')
            xbar = M1 * theta
            ubar = M2 * theta
            V = N * theta
            x0 = x(:,1)
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
            # Terminal Conditions
            if Omegastar == 0:
                CbarTx1 = np.zeros((len(x0),1))
                CbarTx2 = np.zeros((len(x0),1))
            CbarTx1 = []
            CbarTx2 = []
            for i in np.arange(1,Omegastar+1).reshape(-1):
                Sig = np.zeros((Ad.shape[1-1],1))
                for j in np.arange(1,i+1).reshape(-1):
                    Sig = Sig + (Ad + Bd * K) ** (j - 1) * (Bd * M2 * theta - Bd * K * M1 * theta)
                CbarTx1 = np.array([[CbarTx1],[Sig - Xconstraint + 1 / beta]])
                CbarTx2 = np.array([[CbarTx2],[- Sig + Xconstraint_down + 1 / beta]])
            TXConstraints1 = (AbarTx * x0 + BbarTx * hat_u + CbarTx1)
            TXConstraints2 = - (AbarTx * x0 + BbarTx * hat_u) + CbarTx2
            CbarTu1 = M2 * theta - K * M1 * theta - Uconstraint + 1 / beta
            CbarTu2 = - (M2 * theta - K * M1 * theta) + Uconstraint_down + 1 / beta
            for i in np.arange(1,Omegastar - 1+1).reshape(-1):
                Sig = np.zeros((Bd.shape[2-1],1))
                for j in np.arange(1,i+1).reshape(-1):
                    Sig = Sig + K * (Ad + Bd * K) ** (j - 1) * (Bd * M2 * theta - Bd * K * M1 * theta)
                CbarTu1 = np.array([[CbarTu1],[Sig + M2 * theta - K * M1 * theta - Uconstraint + 1 / beta]])
                CbarTu2 = np.array([[CbarTu2],[- (Sig + M2 * theta - K * M1 * theta) + Uconstraint_down + 1 / beta]])
            TUConstraints1 = AbarTu * x0 + BbarTu * hat_u + CbarTu1
            TUConstraints2 = - (AbarTu * x0 + BbarTu * hat_u) + CbarTu2
            TConstraints = np.array([[TXConstraints1],[TXConstraints2],[TUConstraints1],[TUConstraints2]]) <= 0
            TConstraints = np.array([[TConstraints],[M2 * theta <= 0.98 * Uconstraint - 1 / beta],[M2 * theta >= + 0.98 * Uconstraint_down + 1 / beta],[M1 * theta <= 0.98 * Xconstraint - 1 / beta],[M1 * theta >= 0.98 * Xconstraint_down + 1 / beta]])
            Constraint = np.array([[x_pr <= x_up - 1 / beta],[x_pr >= x_down + 1 / beta],[hat_u <= u_up - 1 / beta],[hat_u >= u_down + 1 / beta]])
            Constraints = np.array([[Constraint],[TConstraints]])
            opt = sdpsettings('showprogress',0,'verbose',0)
            sol = optimize(Constraints,sigma,opt)
            clear('x_pr','Constraint','Constraints','x_pr','errU')
            hat_u = double(hat_u)
        AT = DeltaT - 0.05
        if inc == 1:
            u_app = np.zeros((NoI,1))
        else:
            if inc > 1:
                pass
        tic
        #Samplingtime
        MM = 0
        hat_u0 = hat_u
        hatLambda0 = hatLambda
        sigma_hat_u0 = Functions.CostF(r,u_des,x0,inc,Abar,Bbar,M1Bar,M2Bar,hat_u0,QxBar,QuBar,Qn,Qv,Prediction_Horizion,NoS,DeltaT,M1,M2,N,u_app,theta)
        # Primal_dual_gradient_flow
        sigma_values = []
        Num_REAP = 0
        while toc < AT:

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
            TXConstraints1_eps = M1 * theta - 0.98 * Xconstraint + 1 / beta
            TXConstraints2_eps = - M1 * theta + 0.98 * Xconstraint_down + 1 / beta
            TUConstraints1_eps = M2 * theta - 0.98 * Uconstraint + 1 / beta
            TUConstraints2_eps = - M2 * theta + 0.98 * Uconstraint_down + 1 / beta
            CbarTx1 = []
            CbarTx2 = []
            for i in np.arange(1,Omegastar+1).reshape(-1):
                Sig = np.zeros((Ad.shape[1-1],1))
                for j in np.arange(1,i+1).reshape(-1):
                    Sig = Sig + (Ad + Bd * K) ** (j - 1) * (Bd * M2 * theta - Bd * K * M1 * theta)
                CbarTx1 = np.array([[CbarTx1],[Sig - Xconstraint + 1 / beta]])
                CbarTx2 = np.array([[CbarTx2],[- Sig + Xconstraint_down + 1 / beta]])
            TXConstraints1 = AbarTx * x0 + BbarTx * hat_u + CbarTx1
            TXConstraints2 = - (AbarTx * x0 + BbarTx * hat_u) + CbarTx2
            CbarTu1 = M2 * theta - K * M1 * theta - Uconstraint + 1 / beta
            CbarTu2 = - (M2 * theta - K * M1 * theta) + Uconstraint_down + 1 / beta
            for i in np.arange(1,Omegastar - 1+1).reshape(-1):
                Sig = np.zeros((Bd.shape[2-1],1))
                for j in np.arange(1,i+1).reshape(-1):
                    Sig = Sig + K * (Ad + Bd * K) ** (j - 1) * (Bd * M2 * theta - Bd * K * M1 * theta)
                CbarTu1 = np.array([[CbarTu1],[Sig + M2 * theta - K * M1 * theta - Uconstraint + 1 / beta]])
                CbarTu2 = np.array([[CbarTu2],[- (Sig + M2 * theta - K * M1 * theta) + Uconstraint_down + 1 / beta]])
            TUConstraints1 = AbarTu * x0 + BbarTu * hat_u + CbarTu1
            TUConstraints2 = - (AbarTu * x0 + BbarTu * hat_u) + CbarTu2
            AllConstraints.Tx1 = TXConstraints1
            AllConstraints.Tx2 = TXConstraints2
            AllConstraints.Tu1 = TUConstraints1
            AllConstraints.Tu2 = TUConstraints2
            AllConstraints.Tx1_eps = TXConstraints1_eps
            AllConstraints.Tx2_eps = TXConstraints2_eps
            AllConstraints.Tu1_eps = TUConstraints1_eps
            AllConstraints.Tu2_eps = TUConstraints2_eps
            ## X and U and T constraints
            A1x = np.ones((x_pr.shape,x_pr.shape))
            A2x = - 1 * np.ones((x_pr.shape,x_pr.shape))
            B1x = - 1 * np.ones((x_pr.shape,x_pr.shape))
            B2x = 1 * np.ones((x_pr.shape,x_pr.shape))
            Cu1 = np.ones((hat_u.shape,hat_u.shape))
            Cu2 = - 1 * np.ones((hat_u.shape,hat_u.shape))
            D1u = - 1 * np.ones((hat_u.shape,hat_u.shape))
            D2u = np.ones((hat_u.shape,hat_u.shape))
            XConstraints1 = np.multiply(A1x,x_pr) + np.multiply(B1x,x_up) + 1 / beta
            XConstraints2 = np.multiply(A2x,x_pr) + np.multiply(B2x,x_down) + 1 / beta
            UConstraints1 = np.multiply(Cu1,hat_u) + np.multiply(D1u,u_up) + 1 / beta
            UConstraints2 = np.multiply(Cu2,hat_u) + np.multiply(D2u,u_down) + 1 / beta
            AllConstraints.x1 = XConstraints1
            AllConstraints.x2 = XConstraints2
            AllConstraints.u1 = UConstraints1
            AllConstraints.u2 = UConstraints2
            AllConstraints.Ax1 = A1x
            AllConstraints.Ax2 = A2x
            AllConstraints.Bx1 = B1x
            AllConstraints.Bx2 = B2x
            AllConstraints.Cu1 = Cu1
            AllConstraints.Cu2 = Cu2
            AllConstraints.Du1 = D1u
            AllConstraints.Du2 = D2u
            epsilon = eps
            delX = np.amin(np.amax(np.array([[XConstraints1],[XConstraints2]]) - 1 / beta + epsilon),0)
            delU = np.amin(np.amax(np.array([[UConstraints1],[UConstraints2]]) - 1 / beta + epsilon),0)
            delTX = np.amin(np.amax(np.array([[TXConstraints1],[TXConstraints2]]) - 1 / beta + epsilon),0)
            delTU = np.amin(np.amax(np.array([[TUConstraints1],[TUConstraints2]]) - 1 / beta + epsilon),0)
            dell = np.array([[np.abs(delX) / norm(Bd)],[np.abs(delU)],[np.abs(delTX) / (norm(BbarTx))],[np.abs(delTU) / (norm(BbarTu))]])
            dellMM = np.amin(dell)
            # Compute gradients of the objective function
            grad_B_u = Functions.gradient_B_u(Bbar,errU,errX,errTX,QuBar,QxBar,NoS,hatLambda,Prediction_Horizion,AllConstraints,beta,Qn,BbarTx,BbarTu)
            grad_B_lambda = Functions.gradient_B_lambda(AllConstraints,beta)
            ds = 0.001
            ###
            Sigma = dellMM / (ds * np.amax(norm(grad_B_u),eta))
            ###
            Checkpoint = grad_B_lambda + Functions.Phi(hatLambda,grad_B_lambda,hat_lambda)
            flag_gammaD = 0
            while flag_gammaD == 0:

                if np.any((hat_lambda + ds * Sigma * Checkpoint) < 0):
                    Sigma = 0.95 * Sigma
                else:
                    flag_gammaD = 1

            sigma_values = np.array([[sigma_values],[Sigma]])
            hat_u = hat_u - ds * Sigma * grad_B_u
            hat_lambda = hat_lambda + ds * Sigma * (grad_B_lambda + Functions.Phi(hatLambda,grad_B_lambda,hat_lambda))
            hat_lambda_x1 = hat_lambda(np.arange(1,XConstraints1.shape+1))
            hat_lambda_x2 = hat_lambda(np.arange(XConstraints1.shape + 1,2 * XConstraints1.shape+1))
            hat_lambda_u1 = hat_lambda(np.arange(1 + 2 * XConstraints1.shape,2 * XConstraints1.shape + UConstraints1.shape+1))
            hat_lambda_u2 = hat_lambda(np.arange(1 + 2 * XConstraints1.shape + UConstraints1.shape,2 * XConstraints1.shape + 2 * UConstraints1.shape+1))
            hat_lambda_omega_x1 = hat_lambda(np.arange(1 + 2 * XConstraints1.shape + 2 * UConstraints1.shape,2 * XConstraints1.shape + 2 * UConstraints1.shape + TXConstraints1.shape+1))
            hat_lambda_omega_x2 = hat_lambda(np.arange(1 + 2 * XConstraints1.shape + 2 * UConstraints1.shape + TXConstraints1.shape,2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape+1))
            hat_lambda_omega_u1 = hat_lambda(np.arange(1 + 2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape,2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape + TUConstraints1.shape+1))
            hat_lambda_omega_u2 = hat_lambda(np.arange(1 + 2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape + TUConstraints1.shape,2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape + 2 * TUConstraints1.shape+1))
            hat_lambda_omega_x1_eps = hat_lambda(np.arange(1 + 2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape + 2 * TUConstraints1.shape,2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape + 2 * TUConstraints1.shape + TXConstraints1_eps.shape+1))
            hat_lambda_omega_x2_eps = hat_lambda(np.arange(1 + 2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape + 2 * TUConstraints1.shape + TXConstraints1_eps.shape,2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape + 2 * TUConstraints1.shape + 2 * TXConstraints1_eps.shape+1))
            hat_lambda_omega_u1_eps = hat_lambda(np.arange(1 + 2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape + 2 * TUConstraints1.shape + 2 * TXConstraints1_eps.shape,2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape + 2 * TUConstraints1.shape + 2 * TXConstraints1_eps.shape + TUConstraints1_eps.shape+1))
            hat_lambda_omega_u2_eps = hat_lambda(np.arange(1 + 2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape + 2 * TUConstraints1.shape + 2 * TXConstraints1_eps.shape + TUConstraints1_eps.shape,2 * XConstraints1.shape + 2 * UConstraints1.shape + 2 * TXConstraints1.shape + 2 * TUConstraints1.shape + 2 * TXConstraints1_eps.shape + 2 * TUConstraints1_eps.shape+1))
            hatLambda.x1 = hat_lambda_x1
            hatLambda.x2 = hat_lambda_x2
            hatLambda.u1 = hat_lambda_u1
            hatLambda.u2 = hat_lambda_u2
            hatLambda.omega_x1 = hat_lambda_omega_x1
            hatLambda.omega_x2 = hat_lambda_omega_x2
            hatLambda.omega_u1 = hat_lambda_omega_u1
            hatLambda.omega_u2 = hat_lambda_omega_u2
            hatLambda.omega_x1_eps = hat_lambda_omega_x1_eps
            hatLambda.omega_x2_eps = hat_lambda_omega_x2_eps
            hatLambda.omega_u1_eps = hat_lambda_omega_u1_eps
            hatLambda.omega_u2_eps = hat_lambda_omega_u2_eps
            Num_REAP = Num_REAP + 1

        Total_Num_REAP[inc] = Num_REAP
        Sigmas[inc] = sigma_values
        sigma_hat_u = Functions.CostF(r,u_des,x0,inc,Abar,Bbar,M1Bar,M2Bar,hat_u,QxBar,QuBar,Qn,Qv,Prediction_Horizion,NoS,DeltaT,M1,M2,N,u_app,theta)
        #Acceptance,Rejection
        hat_u,hatLambda = Functions.ARMechanism(sigma_hat_u,sigma_hat_u0,hat_u0,hat_u,hatLambda0,hatLambda)
        # #Warm Starting
        hatLambda,hat_u_0 = Functions.Warmstarting(hatLambda,hat_u,NoS,NoI,Prediction_Horizion,M1,M2,x_pr,Omegastar,theta,K)
        u_app[:,inc] = double(hat_u(np.arange(1,NoI+1)))
        x[:,inc + 1] = Ad * x(:,inc) + Bd * u_app(:,inc)
        hat_u = hat_u_0
    
    return x,u_app,Sigmas
    
    return x,u_app,Sigmas