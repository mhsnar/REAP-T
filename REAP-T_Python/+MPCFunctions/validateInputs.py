import numpy as np
    
def validateInputs(A = None,B = None,C = None,D = None,Qx = None,Qu = None,Qv = None,XConstraintsU = None,XConstraintsL = None,UConstraintsU = None,UConstraintsL = None,x0 = None,r = None,nSim = None,DeltaT = None,ModeDropdown = None): 
    # Validate the input matrices for consistency and constraints
    
    # Validate matrix dimensions
    assert_(A.shape[1-1] == A.shape[2-1],'Matrix A must be square')
    assert_(B.shape[1-1] == A.shape[1-1],'Matrix B must have the same number of rows as A')
    assert_(C.shape[2-1] == A.shape[2-1],'Matrix C must have the same number of columns as A')
    assert_(D.shape[1-1] == C.shape[1-1] and D.shape[2-1] == B.shape[2-1],'Matrix D dimensions must match with B and C')
    assert_(Qx.shape[1-1] == A.shape[1-1] and Qx.shape[2-1] == A.shape[1-1],'Matrix Qx dimensions must match with A')
    assert_(Qu.shape[1-1] == B.shape[2-1] and Qu.shape[2-1] == B.shape[2-1],'Matrix Qu dimensions must match with B')
    assert_(Qv.shape[1-1] == C.shape[1-1] and Qv.shape[2-1] == C.shape[1-1],'Matrix Qv dimensions must match with C')
    # Validate Controllability
    controllability_matrix = ctrb(A,B)
    if rank(controllability_matrix) < A.shape[1-1]:
        raise Exception('The pair (A,B) is not controllable. REAP-T cannot proceed with the specified system.')
    else:
        print('System is controllable, ' % ())
    
    # Compute the observability matrix for Prediction-based terminal condition method
    observability_matrix = obsv(A,C)
    if rank(observability_matrix) < A.shape[1-1] and str(ModeDropdown.Value) == str('Prediction-Based'):
        raise Exception('The pair (C, A) is not observable. Please use the Lyapunov-based method to implement the terminal constraint set.')
    
    # Validate initial conditions and desired state
    assert_(x0.shape[1-1] == A.shape[1-1] and x0.shape[2-1] == 1,'Initial conditions x0 must be a column vector matching A dimensions')
    # Validate constraint dimensions
    assert_(XConstraintsU.shape[1-1] == A.shape[1-1] and XConstraintsU.shape[2-1] == 1,'X constraints upper bound must match the state dimension')
    assert_(XConstraintsL.shape[1-1] == A.shape[1-1] and XConstraintsL.shape[2-1] == 1,'X constraints lower bound must match the state dimension')
    assert_(UConstraintsU.shape[1-1] == B.shape[2-1] and UConstraintsU.shape[2-1] == 1,'U constraints upper bound must match the input dimension')
    assert_(UConstraintsL.shape[1-1] == B.shape[2-1] and UConstraintsL.shape[2-1] == 1,'U constraints lower bound must match the input dimension')
    # Validate constraints values
    assert_(np.all(XConstraintsU >= XConstraintsL),'X constraints upper bounds must be greater than or equal to lower bounds')
    assert_(np.all(UConstraintsU >= UConstraintsL),'U constraints upper bounds must be greater than or equal to lower bounds')
    # Validate number of simulation steps and sampling period
    assert_(isnumeric(nSim) and np.isscalar(nSim) and nSim > 0,'# Time Instants (nSim) must be a positive scalar')
    assert_(isnumeric(DeltaT) and np.isscalar(DeltaT) and DeltaT > 0,'Sampling period (DeltaT) must be a positive scalar')
    return
    