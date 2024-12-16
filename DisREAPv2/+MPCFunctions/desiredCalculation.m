
function xbar = desiredCalculation(Ad, Bd, Cd, Dd,NoS,NoI,r)

X=[Ad-eye(length(Ad)) Bd zeros(length(Ad),size(Cd,1));...
    Cd Dd -eye(size(Cd,1),size(Cd,1))];

MN=null(X,"rational");

M1=MN(1:NoS,:);
N=MN(1+NoS+NoI:end,:);
Theta=pinv(N)*r;
xbar=M1*Theta;
end