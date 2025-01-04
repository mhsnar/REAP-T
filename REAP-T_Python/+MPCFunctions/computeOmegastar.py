import numpy as np
    
def computeOmegastar(Ad = None,Bd = None,Cd = None,Dd = None,Xconstraints = None,Uconstraints = None,xbar = None,NoS = None,NoI = None,Qx = None,Qu = None): 
    print('Automatic Calculation of OmegaStar has started!...')
    beta = 100
    __,K,__,__ = idare(Ad,Bd,Qx,Qu,[],[])
    K = - K
    X = np.array([[Ad - np.eye(len(Ad)),Bd,np.zeros((len(Ad),Cd.shape[1-1]))],[Cd,Dd,- np.eye(Cd.shape[1-1],Cd.shape[1-1])]])
    MN = null(X,'rational')
    M1 = MN(np.arange(1,NoS+1),:)
    M2 = MN(np.arange(1 + NoS,+ NoS + NoI+1),:)
    N = MN(np.arange(1 + NoS + NoI,end()+1),:)
    Xconstraint_down = Xconstraints(:,1)
    Xconstraint = Xconstraints(:,end())
    Uconstraint_down = Uconstraints(:,1)
    Uconstraint = Uconstraints(:,end())
    for Omegastar in np.arange(0,100+1).reshape(-1):
        clear('TConstraints_cost','TConstraints','x')
        yalmip('clear')
        theta = pinv(M1) * xbar
        # theta = sdpvar(3,1,'full');
        x = sdpvar(NoS,1,'full')
        AbarTx = np.eye(len(x),len(x))
        for i in np.arange(1,Omegastar+1).reshape(-1):
            AbarTx = np.array([[AbarTx],[(Ad + Bd * K) ** (i)]])
        AbarTu = K
        for i in np.arange(1,Omegastar+1).reshape(-1):
            AbarTu = np.array([[AbarTu],[K * ((Ad + Bd * K)) ** (i)]])
        ## Costraints
        xbar = M1 * theta
        ubar = M2 * theta
        V = N * theta
        CbarTx1 = - Xconstraint + 1 / beta
        CbarTx2 = + Xconstraint_down + 1 / beta
        for i in np.arange(1,Omegastar+1).reshape(-1):
            Sig = np.zeros((Ad.shape[1-1],1))
            Sigg = np.zeros((Ad.shape[2-1],1))
            for j in np.arange(1,i+1).reshape(-1):
                Sig = Sig + (Ad + Bd * K) ** (j - 1) * (Bd * M2 * theta - Bd * K * M1 * theta)
                Sigg = Sigg + (Ad + Bd * K) ** (j) * (Bd * M2 * theta - Bd * K * M1 * theta)
            CbarTx1 = np.array([[CbarTx1],[Sig - Xconstraint + 1 / beta]])
            CbarTx2 = np.array([[CbarTx2],[- Sig + Xconstraint_down + 1 / beta]])
            CbarTx1_T = Sigg + (Bd * M2 * theta - Bd * K * M1 * theta) - Xconstraint + 1 / beta
            CbarTx2_T = - Sigg - (Bd * M2 * theta - Bd * K * M1 * theta) + Xconstraint_down + 1 / beta
        TXConstraints1_omega = AbarTx * x + CbarTx1
        TXConstraints2_omega = - AbarTx * x + CbarTx2
        CbarTu1 = M2 * theta - K * M1 * theta - Uconstraint + 1 / beta
        CbarTu2 = - (M2 * theta - K * M1 * theta) + Uconstraint_down + 1 / beta
        for i in np.arange(1,Omegastar+1).reshape(-1):
            Sig = np.zeros((Bd.shape[2-1],1))
            Sigg = np.zeros((Bd.shape[2-1],1))
            for j in np.arange(1,i+1).reshape(-1):
                Sig = Sig + K * (Ad + Bd * K) ** (j - 1) * (Bd * M2 * theta - Bd * K * M1 * theta)
                Sigg = Sigg + K * (Ad + Bd * K) ** (j) * (Bd * M2 * theta - Bd * K * M1 * theta)
            CbarTu1 = np.array([[CbarTu1],[Sig + M2 * theta - K * M1 * theta - Uconstraint + 1 / beta]])
            CbarTu2 = np.array([[CbarTu2],[- (Sig + M2 * theta - K * M1 * theta) + Uconstraint_down + 1 / beta]])
            CbarTu1_T = np.array([Sigg + K * (Bd * M2 * theta - Bd * K * M1 * theta) + M2 * theta - K * M1 * theta - Uconstraint + 1 / beta])
            CbarTu2_T = np.array([- (Sigg + K * (Bd * M2 * theta - Bd * K * M1 * theta) + M2 * theta - K * M1 * theta) + Uconstraint_down + 1 / beta])
        TUConstraints1_omega = AbarTu * x + CbarTu1
        TUConstraints2_omega = - AbarTu * x + CbarTu2
        TConstraints = np.array([[TXConstraints1_omega],[TXConstraints2_omega],[TUConstraints1_omega],[TUConstraints2_omega]]) <= 0
        # TConstraints = [TConstraints;M2*theta<=0.98*Uconstraint-1/beta;M2*theta>=-0.98*Uconstraint+1/beta;M1*theta<=0.98*Xconstraint-1/beta;M1*theta>=-0.98*Xconstraint+1/beta];
        if Omegastar == 0:
            TConstraints_cost = np.array([[((Ad + Bd * K) ** (Omegastar + 1)) * x + (Bd * M2 * theta - Bd * K * M1 * theta) - Xconstraint + 1 / beta],[- ((Ad + Bd * K) ** (Omegastar + 1)) * x - (Bd * M2 * theta - Bd * K * M1 * theta) + Xconstraint_down + 1 / beta],[K * ((Ad + Bd * K) ** (Omegastar + 1)) * x + K * (Bd * M2 * theta - Bd * K * M1 * theta) + M2 * theta - K * M1 * theta - Uconstraint + 1 / beta],[- K * ((Ad + Bd * K) ** (Omegastar + 1)) * x - (K * (Bd * M2 * theta - Bd * K * M1 * theta) + M2 * theta - K * M1 * theta) + Uconstraint_down + 1 / beta]])
        else:
            if Omegastar == 1:
                TConstraints_cost = np.array([[((Ad + Bd * K) ** (Omegastar + 1)) * x + CbarTx1_T],[- ((Ad + Bd * K) ** (Omegastar + 1)) * x + CbarTx2_T],[K * ((Ad + Bd * K) ** (Omegastar + 1)) * x + K * (Ad + Bd * K) * (Bd * M2 * theta - Bd * K * M1 * theta) + K * (Bd * M2 * theta - Bd * K * M1 * theta) + M2 * theta - K * M1 * theta - Uconstraint + 1 / beta],[- K * ((Ad + Bd * K) ** (Omegastar + 1)) * x - (K * (Ad + Bd * K) * (Bd * M2 * theta - Bd * K * M1 * theta) + K * (Bd * M2 * theta - Bd * K * M1 * theta) + M2 * theta - K * M1 * theta) + Uconstraint_down + 1 / beta]])
            else:
                TConstraints_cost = np.array([[((Ad + Bd * K) ** (Omegastar + 1)) * x + CbarTx1_T],[- ((Ad + Bd * K) ** (Omegastar + 1)) * x + CbarTx2_T],[K * ((Ad + Bd * K) ** (Omegastar + 1)) * x + CbarTu1_T],[- K * ((Ad + Bd * K) ** (Omegastar + 1)) * x + CbarTu2_T]])
        TXConstraints1_eps = M1 * theta - 0.98 * Xconstraint + 1 / beta
        TXConstraints2_eps = - M1 * theta + 0.98 * Xconstraint_down + 1 / beta
        TUConstraints1_eps = M2 * theta - 0.98 * Uconstraint + 1 / beta
        TUConstraints2_eps = - M2 * theta + 0.98 * + Uconstraint_down + 1 / beta
        # TConstraints_cost = [TConstraints_cost;TXConstraints1_eps;TXConstraints2_eps;TUConstraints1_eps;TUConstraints2_eps];    # # If we have theta
        for i in np.arange(1,len(TConstraints_cost)+1).reshape(-1):
            cons = TConstraints_cost(i)
            sigma = - cons
            options = sdpsettings('solver','fmincon','showprogress',0,'verbose',0)
            optimize(TConstraints,sigma,options)
            J[i] = double(- sigma)
            if J(i) > 0:
                break
        if J <= 0:
            break
    
    print('Omegastar: %f\n' % (Omegastar))
    print('Automatic Calculation of OmegaStar ended!')
    return Omegastar
    
    return Omegastar