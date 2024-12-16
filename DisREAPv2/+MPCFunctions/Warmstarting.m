function [hatLambda,hat_u_0] = Warmstarting(hatLambda,hat_u,NoS,NoI,Prediction_Horizion,M1,M2,x_pr,Omegastar,theta,K)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here
   %Warm Starting
    hat_u_0=[];
    hat_lambda_x1_0=[];
    hat_lambda_x2_0=[];
    hat_lambda_u1_0=[];
    hat_lambda_u2_0=[];
    hat_lambda_omega_x1_0=[];
    hat_lambda_omega_x2_0=[];
    hat_lambda_omega_u1_0=[];
    hat_lambda_omega_u2_0=[];
    % hat_lambda_omega_x1_eps_0=[];
    % hat_lambda_omega_x2_eps_0=[];
    % hat_lambda_omega_u1_eps_0=[];
    % hat_lambda_omega_u2_eps_0=[];

    hat_lambda_x1=hatLambda.x1;
    hat_lambda_x2=hatLambda.x2;
    hat_lambda_u1=hatLambda.u1;
    hat_lambda_u2=hatLambda.u2;
    hat_lambda_omega_x1=hatLambda.omega_x1;
    hat_lambda_omega_x2=hatLambda.omega_x2;
    hat_lambda_omega_u1=hatLambda.omega_u1;
    hat_lambda_omega_u2=hatLambda.omega_u2;
    hat_lambda_omega_x1_eps=hatLambda.omega_x1_eps;
    hat_lambda_omega_x2_eps=hatLambda.omega_x2_eps;
    hat_lambda_omega_u1_eps=hatLambda.omega_u1_eps;
    hat_lambda_omega_u2_eps=hatLambda.omega_u2_eps;




    for p=1:Prediction_Horizion-1
        hat_u_0=[hat_u_0;hat_u(NoI*p+1:NoI*p+NoI)];
        hat_lambda_x1_0=[hat_lambda_x1_0;hat_lambda_x1(NoS*p+1:NoS*p+NoS)];
        hat_lambda_x2_0=[hat_lambda_x2_0;hat_lambda_x2(NoS*p+1:NoS*p+NoS)];
        hat_lambda_u1_0=[hat_lambda_u1_0;hat_lambda_u1(NoI*p+1:NoI*p+NoI)];
        hat_lambda_u2_0=[hat_lambda_u2_0;hat_lambda_u2(NoI*p+1:NoI*p+NoI)];
    end
    for p=1:Omegastar-1
        hat_lambda_omega_x1_0=[hat_lambda_omega_x1_0;hat_lambda_omega_x1(NoS*p+1:NoS*(p+1))];
        hat_lambda_omega_x2_0=[hat_lambda_omega_x2_0;hat_lambda_omega_x2(NoS*p+1:NoS*(p+1))];
        hat_lambda_omega_u1_0=[hat_lambda_omega_u1_0;hat_lambda_omega_u1(NoI*p+1:NoI*(p+1))];
        hat_lambda_omega_u2_0=[hat_lambda_omega_u2_0;hat_lambda_omega_u2(NoI*p+1:NoI*(p+1))];

    end
    %the same

    hat_lambda_omega_x1_eps_0=hat_lambda_omega_x1_eps;
    hat_lambda_omega_x2_eps_0=hat_lambda_omega_x2_eps;
    hat_lambda_omega_u1_eps_0=hat_lambda_omega_u1_eps;
    hat_lambda_omega_u2_eps_0=hat_lambda_omega_u2_eps;


    hat_u_0=[hat_u_0; M2*theta+K*(x_pr(NoS*Prediction_Horizion-NoS+1:NoS*Prediction_Horizion)-M1*theta)];
    hat_lambda_x1_0=[hat_lambda_x1_0;hat_lambda_x1(NoS*Prediction_Horizion-NoS+1:NoS*Prediction_Horizion)];
    hat_lambda_x2_0=[hat_lambda_x2_0;hat_lambda_x2(NoS*Prediction_Horizion-NoS+1:NoS*Prediction_Horizion)];
    hat_lambda_u1_0=[hat_lambda_u1_0;hat_lambda_u1(NoI*Prediction_Horizion-NoI+1:NoI*Prediction_Horizion)];
    hat_lambda_u2_0=[hat_lambda_u2_0;hat_lambda_u2(NoI*Prediction_Horizion-NoI+1:NoI*Prediction_Horizion)];
    hat_lambda_omega_x1_0=[hat_lambda_omega_x1_0;hat_lambda_omega_x1(NoS*Omegastar-NoS+1:NoS*Omegastar)];
    hat_lambda_omega_x2_0=[hat_lambda_omega_x2_0;hat_lambda_omega_x2(NoS*Omegastar-NoS+1:NoS*Omegastar)];
    hat_lambda_omega_u1_0=[hat_lambda_omega_u1_0;hat_lambda_omega_u1(NoI*Omegastar-NoI+1:NoI*Omegastar)];
    hat_lambda_omega_u2_0=[hat_lambda_omega_u2_0;hat_lambda_omega_u2(NoI*Omegastar-NoI+1:NoI*Omegastar)];
    hatLambda.x1=hat_lambda_x1_0;
    hatLambda.x2=hat_lambda_x2_0;
    hatLambda.u1=hat_lambda_u1_0;
    hatLambda.u2=hat_lambda_u2_0;
    hatLambda.omega_x1=hat_lambda_omega_x1_0;
    hatLambda.omega_x2=hat_lambda_omega_x2_0;
    hatLambda.omega_u1=hat_lambda_omega_u1_0;
    hatLambda.omega_u2=hat_lambda_omega_u2_0;
    hatLambda.omega_x1_eps=hat_lambda_omega_x1_eps_0;
    hatLambda.omega_x2_eps=hat_lambda_omega_x2_eps_0;
    hatLambda.omega_u1_eps=hat_lambda_omega_u1_eps_0;
    hatLambda.omega_u2_eps=hat_lambda_omega_u2_eps_0;
end