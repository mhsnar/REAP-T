function validateInputs(A, B, C, D, Qx, Qu, Qv, XConstraintsU, XConstraintsL, UConstraintsU, UConstraintsL, x0, r, nSim, DeltaT,ModeDropdown)
    % Validate the input matrices for consistency and constraints
    
    % Validate matrix dimensions
    assert(size(A, 1) == size(A, 2), 'Matrix A must be square');
    assert(size(B, 1) == size(A, 1), 'Matrix B must have the same number of rows as A');
    assert(size(C, 2) == size(A, 2), 'Matrix C must have the same number of columns as A');
    assert(size(D, 1) == size(C, 1) && size(D, 2) == size(B, 2), 'Matrix D dimensions must match with B and C');
    assert(size(Qx, 1) == size(A, 1) && size(Qx, 2) == size(A, 1), 'Matrix Qx dimensions must match with A');
    assert(size(Qu, 1) == size(B, 2) && size(Qu, 2) == size(B, 2), 'Matrix Qu dimensions must match with B');
    assert(size(Qv, 1) == size(C, 1) && size(Qv, 2) == size(C, 1), 'Matrix Qv dimensions must match with C');
    
    % Validate Controllability
    controllability_matrix = ctrb(A, B);
    if rank(controllability_matrix) < size(A, 1)
        error('The pair (A,B) is not controllable. REAP-T cannot proceed with the specified system.');
    else
        fprintf('System is controllable, ');
    end

    % Compute the observability matrix for Prediction-based terminal
    % condition method
    observability_matrix = obsv(A, C);
    if rank(observability_matrix) < size(A, 1) && strcmp(ModeDropdown.Value, 'Prediction-Based') 
        error('The pair (C, A) is not observable. Please use the Lyapunov-based method to implement the terminal constraint set.');
    end
    
    % Validate initial conditions and desired state
    assert(size(x0, 1) == size(A, 1) && size(x0, 2) == 1, 'Initial conditions x0 must be a column vector matching A dimensions');
    % assert(size(r, 1) == size(C, 1) && size(r, 2) == 1, 'Desired reference r must be a column vector matching C rows');
    % assert(size(xbar, 1) == size(A, 1) && size(xbar, 2) == 1, 'Desired equilibrium point r must be a column vector matching C rows');
    
    % Validate constraint dimensions
    assert(size(XConstraintsU, 1) == size(A, 1) && size(XConstraintsU, 2) == 1, 'X constraints upper bound must match the state dimension');
    assert(size(XConstraintsL, 1) == size(A, 1) && size(XConstraintsL, 2) == 1, 'X constraints lower bound must match the state dimension');
    assert(size(UConstraintsU, 1) == size(B, 2) && size(UConstraintsU, 2) == 1, 'U constraints upper bound must match the input dimension');
    assert(size(UConstraintsL, 1) == size(B, 2) && size(UConstraintsL, 2) == 1, 'U constraints lower bound must match the input dimension');
    
    % Validate constraints values
    assert(all(XConstraintsU >= XConstraintsL), 'X constraints upper bounds must be greater than or equal to lower bounds');
    assert(all(UConstraintsU >= UConstraintsL), 'U constraints upper bounds must be greater than or equal to lower bounds');
    
    % Validate number of simulation steps and sampling period
    assert(isnumeric(nSim) && isscalar(nSim) && nSim > 0, '# Time Instants (nSim) must be a positive scalar');
    assert(isnumeric(DeltaT) && isscalar(DeltaT) && DeltaT > 0, 'Sampling period (DeltaT) must be a positive scalar');
end
