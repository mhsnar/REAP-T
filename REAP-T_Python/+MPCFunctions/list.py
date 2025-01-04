import numpy as np
    
def list(AllConstraints = None): 
    #listConstraints Displays constraints in a readable format.
    XUB = AllConstraints.XUB
    XLB = AllConstraints.XLB
    UUB = AllConstraints.UUB
    ULB = AllConstraints.ULB
    NoI = AllConstraints.NoI
    NoS = AllConstraints.NoS
    print('State Constraints:')
    for i in np.arange(1,NoS+1).reshape(-1):
        # Display the constraint
        print(np.array(['State Constraint ',num2str(i),': ','x',(num2str(i)),' <= ',num2str(XUB(i))]))
    
    for i in np.arange(1,NoS+1).reshape(-1):
        # Display the constraint
        print(np.array(['State Constraint ',num2str(NoS + i),': ','x',num2str(i),' >= ',num2str(XLB(i))]))
    
    # Display general constraints
    print('Input Constraints:')
    for i in np.arange(1,NoI+1).reshape(-1):
        # Display the constraint
        print(np.array(['Input Constraint ',num2str(i),': ','u',num2str(i),' <= ',num2str(UUB(i))]))
    
    for i in np.arange(1,NoI+1).reshape(-1):
        # Display the constraint
        print(np.array(['Input Constraint ',num2str(NoI + i),': ','u',num2str(i),' >= ',num2str(ULB(i))]))
    
    return
    