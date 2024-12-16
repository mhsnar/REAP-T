function list(AllConstraints)
%listConstraints Displays constraints in a readable format.
XUB=AllConstraints.XUB;
XLB=AllConstraints.XLB;
UUB=AllConstraints.UUB;
ULB=AllConstraints.ULB;
NoI=AllConstraints.NoI;
NoS=AllConstraints.NoS;

disp('State Constraints:');
for i = 1:NoS

    % Display the constraint
    disp(['State Constraint ', num2str(i), ': ', 'x' ,(num2str(i)), ' <= ',num2str(XUB(i))]);
end
for i =1:NoS

    % Display the constraint
    disp(['State Constraint ', num2str(NoS+i), ': ', 'x' ,num2str(i), ' >= ',num2str(XLB(i))]);
end

% Display general constraints
disp('Input Constraints:');
for i = 1:NoI

    % Display the constraint
    disp(['Input Constraint ', num2str(i), ': ', 'u' ,num2str(i), ' <= ',num2str(UUB(i))]);
end
for i = 1:NoI

    % Display the constraint
    disp(['Input Constraint ', num2str(NoI+i), ': ', 'u' ,num2str(i), ' >= ',num2str(ULB(i))]);
end
end
