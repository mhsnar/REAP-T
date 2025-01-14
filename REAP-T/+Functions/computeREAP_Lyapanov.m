function [x, u_app,Sigmas] = computeREAP_Lyapanov(Ad, Bd, Cd,Dd,Xconstraints,Uconstraints,x0,Xbar, NoS, NoI, NoO, Qx, Qu, Qv, DeltaT, Prediction_Horizion, Omegastar, n,C)
% Compute MPC control action
% Implement the MPC logic here based on the provided code
disp('REAP has started!...');
% Initialize variables

u_des=zeros(NoI,1);
% Initial conditions and reference

x(:, 1) = [x0;zeros(NoI,1)];
r=Cd(:,1:NoS-NoI)*Xbar;
% Discretized system matrices for augmented system
Xconstraint_down=Xconstraints(:,1);
Xconstraint=Xconstraints(:,end);

Uconstraint_down=Uconstraints(:,1);
Uconstraint=Uconstraints(:,end);

% State and control constraints
% Xconstraint = [Xconstraint;Uconstraint];
% Xconstraint_down=[Xconstraint_down;Uconstraint_down];
Xconstraint = [Xconstraint;1000* ones(NoI,1)];
Xconstraint_down=[Xconstraint_down;-1000* ones(NoI,1)];

% Extended matrices for prediction horizon
QxBar = blkdiag(kron(eye(Prediction_Horizion), Qx));
QuBar = blkdiag(kron(eye(Prediction_Horizion), Qu));
Qx=[Qx zeros(size(Qx,1),size(Qu,1));zeros(size(Qu,1),size(Qx,1)) Qu];

[Qn,K,~,~] = idare(Ad,Bd,Qx,Qu,[],[]);
K=-K;
psi=dlyap((Ad+Bd*K)',eye(NoS));

% Placeholder for control input
u_app = zeros(NoI, n);
X=[Ad-eye(length(Ad)) Bd zeros(length(Ad),size(Cd,1));...
    Cd Dd -eye(size(Cd,1),size(Cd,1))];

MN=null(X,"rational");


M1=MN(1:NoS-NoI,:);
M2=MN(1+NoS:+NoS+NoI,:);
M1=[M1;M2];

N=MN(1+NoS+NoI:end,:);



% Calculating x_pr:
Abar=Ad;
for i=2:Prediction_Horizion
    Abar=[Abar;Ad^(i)];
end

[o, m] = size(Bd);
Bbar = zeros(o * Prediction_Horizion, m * Prediction_Horizion);

for i = 1:Prediction_Horizion
    for j = 1:i
        A_power = Ad^(i-j);

        % Element-wise multiplication using matrix multiplication
        block = A_power * Bd;

        % Calculate the indices for insertion
        row_indices = (i-1)*o + 1 : i*o;
        col_indices = (j-1)*m + 1 : j*m;

        % Insert the block into the appropriate position in Bbar
        Bbar(row_indices, col_indices) = block;
    end
end

beta=100;



x_up=[];
u_up=[];
u_down=[];
x_down=[];
for i=1:Prediction_Horizion
    u_up=[u_up;Uconstraint];
    u_down=[u_down;Uconstraint_down];

    x_up=[x_up;Xconstraint];
    x_down=[x_down;Xconstraint_down];
end
% u_down=-u_up;
% x_down=-x_up;

% Create QuBar using blkdiag
QuBar = blkdiag(kron(eye(Prediction_Horizion), Qu));
QxBar = blkdiag(kron(eye(Prediction_Horizion), Qx));


M1Bar=M1;
M2Bar=M2;
for i=1:Prediction_Horizion-1
    M1Bar=[M1Bar;M1];
    M2Bar=[M2Bar;M2];
end



Number_of_Constraints=(Prediction_Horizion+2)*(2*size(Xconstraint,1)+2*size(Uconstraint,1));


hat_lambda=ones(Number_of_Constraints,1);

hat_lambda_x1=ones(size(Xconstraint,1)*(Prediction_Horizion),1);
hat_lambda_x2=ones(size(Xconstraint,1)*Prediction_Horizion,1);
hat_lambda_u1=ones(size(Uconstraint,1)*Prediction_Horizion,1);
hat_lambda_u2=ones(size(Uconstraint,1)*Prediction_Horizion,1);

hat_lambda_omega_x1=ones(size(Xconstraint,1),1);
hat_lambda_omega_x2=ones(size(Xconstraint,1),1);
hat_lambda_omega_u1=ones(size(Uconstraint,1),1);
hat_lambda_omega_u2=ones(size(Uconstraint,1),1);

hat_lambda_omega_x1_eps=ones(size(Xconstraint,1),1);
hat_lambda_omega_x2_eps=ones(size(Xconstraint,1),1);
hat_lambda_omega_u1_eps=ones(size(Uconstraint,1),1);
hat_lambda_omega_u2_eps=ones(size(Uconstraint,1),1);

hat_u=zeros(Prediction_Horizion*size(Uconstraint,1),1);
hatLambda.x1=hat_lambda_x1;
hatLambda.x2=hat_lambda_x2;
hatLambda.u1=hat_lambda_u1;
hatLambda.u2=hat_lambda_u2;
hatLambda.omega_x1=hat_lambda_omega_x1;
hatLambda.omega_x2=hat_lambda_omega_x2;
hatLambda.omega_u1=hat_lambda_omega_u1;
hatLambda.omega_u2=hat_lambda_omega_u2;
hatLambda.omega_x1_eps=hat_lambda_omega_x1_eps;
hatLambda.omega_x2_eps=hat_lambda_omega_x2_eps;
hatLambda.omega_u1_eps=hat_lambda_omega_u1_eps;
hatLambda.omega_u2_eps=hat_lambda_omega_u2_eps;
%%
flag=0;
f=1;
eta=0.001;
sign=[];
Sigmas = [];
Sigmass = [];
sigma_values=[];
Total_Num_REAP=zeros(n,1);
for inc=1:n

     theta=pinv(MN(1:NoS-NoI,:))*Xbar;


    xbar=M1*theta;
    ubar=M2*theta;
    V=N*theta;

    %
    if inc==1
        x0=x(:,inc);

    elseif inc>1
        % x0=[x(1,inc);(x(1,inc)-x(1,inc-1))/DeltaT;x(3,inc);(x(3,inc)-x(3,inc-1))/DeltaT;x(5,inc);(x(5,inc)-x(5,inc-1))/DeltaT;u_app(:,inc-1)];
        x0=x(:,inc);
    end

    %% Terminal Conditions
    if inc==1 %Optimization
        yalmip('clear')
        % theta = sdpvar(3,1,'full');
        % theta=[r(1) r(3) r(5)]';
        hat_u = sdpvar(NoI*Prediction_Horizion,1,'full');
        xbar=M1*theta;
        ubar=M2*theta;
        V=N*theta;
        x0=x(:,1);
        x_pr =Abar*x0+Bbar*hat_u ;

        errV=V-r;
        ErrorV=(errV'*Qv*errV);
        norU=ubar-u_des;
        NormU= (norU'*norU);
        errU = hat_u - M2Bar*theta;
        ErrorU = errU' * QuBar * errU;
        errX = x_pr - M1Bar*theta;
        ErrorX = errX' * QxBar * errX;
        errTX = x_pr(NoS * Prediction_Horizion - (NoS-1):NoS * Prediction_Horizion) - xbar;
        ErrorTX = errTX' * Qn * errTX;
        sigma = ErrorV+ErrorU + ErrorX + ErrorTX + NormU;
        % Terminal Conditions
        invpsi=inv(psi);
        err=x_pr(size(Ad,1)*Prediction_Horizion-(size(Ad,1)-1):size(Ad,1)*Prediction_Horizion)-xbar;
        LyapanovF=(err)'*psi*(err);

        TXConstraints1=[];
        TXConstraints2=[];
        coeff=eye(NoS);

        for incc=1:NoS
            TXConstraints1=[TXConstraints1;LyapanovF-((xbar(incc)-Xconstraint(incc))^2/(coeff(incc,:)*invpsi*coeff(incc,:)'))+1/beta];
            TXConstraints2=[TXConstraints2;LyapanovF-((-xbar(incc)+Xconstraint_down(incc))^2/(coeff(incc,:)*invpsi*coeff(incc,:)'))+1/beta];
        end

        TUConstraints1=[];
        TUConstraints2=[];
        for incc=1:NoI
            TUConstraints1 = [TUConstraints1;LyapanovF-((ubar(incc)-Uconstraint(incc))^2/(K(incc,:)*invpsi*K(incc,:)'))+1/beta];
            TUConstraints2 = [TUConstraints2;LyapanovF-((-ubar(incc)+Uconstraint_down(incc))^2/(K(incc,:)*invpsi*K(incc,:)'))+1/beta];
        end

        TConstraints=[TXConstraints1;TXConstraints2;TUConstraints1;TUConstraints2]<=0;
        TConstraints = [TConstraints;M2*theta<=0.98*Uconstraint-1/beta;M2*theta>=+0.98*Uconstraint_down+1/beta;M1*theta<=0.98*Xconstraint-1/beta;M1*theta>=0.98*Xconstraint_down+1/beta];

        Constraint=[x_pr<=x_up-1/beta;x_pr>=x_down+1/beta;hat_u<=u_up-1/beta;hat_u>=u_down+1/beta];



        Constraints=[Constraint;TConstraints];
        opt=sdpsettings('solver','fmincon','showprogress',0,'verbose',0);
        sol=optimize(Constraints,sigma,opt);
        if sol.problem~= 0
           
        error('The specified prediction horizon length is insufficient for implementing the Lyapunov-based method. Please increase the prediction horizon length.');
        end

        clear x_pr Constraint Constraints x_pr errU
        hat_u=double(hat_u);
    else
    end

    AT=DeltaT-0.05;   %Available Time for Primal_dual_gradient_flow

    if inc==1
        u_app=zeros(NoI,1);
    elseif inc>1
        %
    end

    tic;
    %Samplingtime

    hat_u0=hat_u;
    hatLambda0=hatLambda;
    sigma_hat_u0 = Functions.CostF(r,u_des,x0,inc,Abar,Bbar,M1Bar,M2Bar,hat_u0,QxBar,QuBar,Qn,Qv,Prediction_Horizion,NoS,DeltaT,M1,M2,N,u_app,theta);
    % Primal_dual_gradient_flow
    sigma_values=[];
    Num_REAP=0;
    
    while toc<AT


        x_pr =Abar*x0+Bbar*hat_u ;
        errV=V-r;
        ErrorV=(errV'*Qv*errV);
        norU=ubar-u_des;
        NormU= (norU'*norU);
        errU = hat_u - M2Bar*theta;
        ErrorU = errU' * QuBar * errU;
        errX = x_pr - M1Bar*theta;
        ErrorX = errX' * QxBar * errX;
        errTX = x_pr(NoS * Prediction_Horizion - (NoS-1):NoS * Prediction_Horizion) - xbar;
        ErrorTX = errTX' * Qn * errTX;

        TXConstraints1_eps= M1*theta-0.98*Xconstraint+1/beta;
        TXConstraints2_eps= -M1*theta+0.98*Xconstraint_down+1/beta;
        TUConstraints1_eps=M2*theta-0.98*Uconstraint+1/beta;
        TUConstraints2_eps=- M2*theta+0.98*Uconstraint_down+1/beta;


        invpsi=inv(psi);
        err=x_pr(size(Ad,1)*Prediction_Horizion-(size(Ad,1)-1):size(Ad,1)*Prediction_Horizion)-xbar;
        LyapanovF=(err)'*psi*(err);

        TXConstraints1=[];
        TXConstraints2=[];
        coeff=eye(NoS);

        for incc=1:NoS
            TXConstraints1=[TXConstraints1;LyapanovF-((xbar(incc)-Xconstraint(incc))^2/(coeff(incc,:)*invpsi*coeff(incc,:)'))+1/beta];
            TXConstraints2=[TXConstraints2;LyapanovF-((-xbar(incc)+Xconstraint_down(incc))^2/(coeff(incc,:)*invpsi*coeff(incc,:)'))+1/beta];
        end

        TUConstraints1=[];
        TUConstraints2=[];
        for incc=1:NoI
            TUConstraints1 = [TUConstraints1;LyapanovF-((ubar(incc)-Uconstraint(incc))^2/(K(incc,:)*invpsi*K(incc,:)'))+1/beta];
            TUConstraints2 = [TUConstraints2;LyapanovF-((-ubar(incc)+Uconstraint_down(incc))^2/(K(incc,:)*invpsi*K(incc,:)'))+1/beta];
        end

        AllConstraints.Tx1=TXConstraints1;
        AllConstraints.Tx2=TXConstraints2;
        AllConstraints.Tu1=TUConstraints1;
        AllConstraints.Tu2=TUConstraints2;
        AllConstraints.Tx1_eps=TXConstraints1_eps;
        AllConstraints.Tx2_eps=TXConstraints2_eps;
        AllConstraints.Tu1_eps=TUConstraints1_eps;
        AllConstraints.Tu2_eps=TUConstraints2_eps;
        %% X and U and T constraints

        A1x=ones(size(x_pr));
        A2x=-1*ones(size(x_pr));
        B1x=-1*ones(size(x_pr));
        B2x=1*ones(size(x_pr));
        Cu1=ones(size(hat_u));
        Cu2=-1*ones(size(hat_u));
        D1u=-1*ones(size(hat_u));
        D2u=ones(size(hat_u));

        XConstraints1=A1x.*x_pr+B1x.*x_up+1/beta;
        XConstraints2=A2x.*x_pr+B2x.*x_down+1/beta;
        UConstraints1=Cu1.*hat_u+D1u.*u_up+1/beta;
        UConstraints2=Cu2.*hat_u+D2u.*u_down+1/beta;

        AllConstraints.x1=XConstraints1;
        AllConstraints.x2=XConstraints2;
        AllConstraints.u1=UConstraints1;
        AllConstraints.u2=UConstraints2;
        AllConstraints.Ax1=A1x;
        AllConstraints.Ax2=A2x;
        AllConstraints.Bx1=B1x;
        AllConstraints.Bx2=B2x;
        AllConstraints.Cu1=Cu1;
        AllConstraints.Cu2=Cu2;
        AllConstraints.Du1=D1u;
        AllConstraints.Du2=D2u;

        epsilon=eps;
        delX=min(max([XConstraints1;XConstraints2]-1/beta+epsilon),0);
        delU=min(max([UConstraints1;UConstraints2]-1/beta+epsilon),0);
        dell=[abs(delX)/norm(Bd);abs(delU)];
        dellMM=min(dell);

        % Compute gradients of the objective function
        grad_B_u = Functions.gradient_B_u_Lyapanov(Bbar, errU,errX,errTX, QuBar,QxBar, NoS,hatLambda, Prediction_Horizion,AllConstraints,beta,Qn,psi);
        grad_B_lambda = Functions.gradient_B_lambda( AllConstraints, beta);
        ds=0.001;

        %%%
        Sigma=dellMM/(ds*max(norm(grad_B_u),eta));
        %%%

        Checkpoint=grad_B_lambda+Functions.Phi(hatLambda,grad_B_lambda,hat_lambda);
        
        %%
        flag_gammaD=0;
        while flag_gammaD==0
            if any((hat_lambda+ds*Sigma*Checkpoint)<0) || any(Functions.TconUpdate_Lyap(hat_u,ds,Sigma,grad_B_u,Abar,Bbar,x0,psi,Ad,Prediction_Horizion,xbar,ubar,NoS,NoI,Xconstraint,Uconstraint,Uconstraint_down,Xconstraint_down,beta,K)>0)
                Sigma=0.95*Sigma;
            else
                flag_gammaD=1;
            end
        end

        sigma_values=[sigma_values;Sigma];
        
        %% Update
        hat_u= hat_u- ds*Sigma*grad_B_u;
        hat_lambda=hat_lambda+ds*Sigma*(grad_B_lambda+ Functions.Phi(hatLambda,grad_B_lambda,hat_lambda));

        hat_lambda_x1=hat_lambda(1: size(XConstraints1));
        hat_lambda_x2=hat_lambda(size(XConstraints1)+1:2*size(XConstraints1));
        hat_lambda_u1=hat_lambda(1+2*size(XConstraints1):2*size(XConstraints1)+size(UConstraints1));
        hat_lambda_u2=hat_lambda(1+2*size(XConstraints1)+size(UConstraints1):2*size(XConstraints1)+2*size(UConstraints1));
        hat_lambda_omega_x1=hat_lambda(1+2*size(XConstraints1)+2*size(UConstraints1):2*size(XConstraints1)+2*size(UConstraints1)+size(TXConstraints1));
        hat_lambda_omega_x2=hat_lambda(1+2*size(XConstraints1)+2*size(UConstraints1)+size(TXConstraints1):2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1));
        hat_lambda_omega_u1=hat_lambda(1+2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1):2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1)+size(TUConstraints1));
        hat_lambda_omega_u2=hat_lambda(1+2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1)+size(TUConstraints1):2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1)+2*size(TUConstraints1));
        hat_lambda_omega_x1_eps=hat_lambda(1+2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1)+2*size(TUConstraints1):2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1)+2*size(TUConstraints1)+size(TXConstraints1_eps));
        hat_lambda_omega_x2_eps=hat_lambda(1+2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1)+2*size(TUConstraints1)+size(TXConstraints1_eps):2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1)+2*size(TUConstraints1)+2*size(TXConstraints1_eps));
        hat_lambda_omega_u1_eps=hat_lambda(1+2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1)+2*size(TUConstraints1)+2*size(TXConstraints1_eps):2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1)+2*size(TUConstraints1)+2*size(TXConstraints1_eps)+size(TUConstraints1_eps));
        hat_lambda_omega_u2_eps=hat_lambda(1+2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1)+2*size(TUConstraints1)+2*size(TXConstraints1_eps)+size(TUConstraints1_eps):2*size(XConstraints1)+2*size(UConstraints1)+2*size(TXConstraints1)+2*size(TUConstraints1)+2*size(TXConstraints1_eps)+2*size(TUConstraints1_eps));

        hatLambda.x1=hat_lambda_x1;
        hatLambda.x2=hat_lambda_x2;
        hatLambda.u1=hat_lambda_u1;
        hatLambda.u2=hat_lambda_u2;
        hatLambda.omega_x1=hat_lambda_omega_x1;
        hatLambda.omega_x2=hat_lambda_omega_x2;
        hatLambda.omega_u1=hat_lambda_omega_u1;
        hatLambda.omega_u2=hat_lambda_omega_u2;
        hatLambda.omega_x1_eps=hat_lambda_omega_x1_eps;
        hatLambda.omega_x2_eps=hat_lambda_omega_x2_eps;
        hatLambda.omega_u1_eps=hat_lambda_omega_u1_eps;
        hatLambda.omega_u2_eps=hat_lambda_omega_u2_eps;


        Num_REAP=Num_REAP+1;
    end
    Total_Num_REAP(inc)=Num_REAP;
    Sigmas{inc} = sigma_values;  % Store all Sigma values for the current increment

    sigma_hat_u = Functions.CostF(r,u_des,x0,inc,Abar,Bbar,M1Bar,M2Bar,hat_u,QxBar,QuBar,Qn,Qv,Prediction_Horizion,NoS,DeltaT,M1,M2,N,u_app,theta);
    %Acceptance,Rejection
    [hat_u,hatLambda] = Functions.ARMechanism(sigma_hat_u,sigma_hat_u0,hat_u0,hat_u,hatLambda0,hatLambda);
    %Warm Starting
    [hatLambda,hat_u_0] = Functions.Warmstarting_Lyapunov(hatLambda,hat_u,NoS,NoI,Prediction_Horizion,M1,M2,x_pr,Omegastar,theta,K);
   


    u_app(:,inc)=double(hat_u(1:NoI));
    x(:,inc+1)=Ad*x(:,inc)+Bd*u_app(:,inc);
    %Update
    hat_u=hat_u_0;

end


end
