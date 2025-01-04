import numpy as np
    
def initialize(A = None,B = None,C = None,D = None,DeltaT = None): 
    # Initialize and discretize system matrices
    G = ss(A,B,C,D)
    Gd = c2d(G,DeltaT)
    Ad = Gd.A
    Bd = Gd.B
    Cd = Gd.C
    Dd = Gd.D
    NoS = Ad.shape[1-1]
    NoI = Bd.shape[2-1]
    NoO = Cd.shape[1-1]
    Ad = np.array([[Ad,Bd],[np.zeros((NoI,NoS)),np.zeros((NoI,NoI))]])
    Bd = np.array([[np.zeros((NoS,NoI))],[np.eye(NoI)]])
    Cd = np.array([Cd,Dd])
    NoS = Ad.shape[1-1]
    NoI = Bd.shape[2-1]
    NoO = Cd.shape[1-1]
    return Ad,Bd,Cd,Dd,NoS,NoI,NoO
    
    return Ad,Bd,Cd,Dd,NoS,NoI,NoO