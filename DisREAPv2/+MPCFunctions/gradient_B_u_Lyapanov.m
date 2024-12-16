function grad_B_u = gradient_B_u_Lyapanov(Bbar, errU,errX,errTX, QuBar,QxBar, NoS,hatLambda, Prediction_Horizion,AllConstraints,beta,Qn,psi)
        XConstraints1=AllConstraints.x1;
        XConstraints2=AllConstraints.x2;
        UConstraints1=AllConstraints.u1;
        UConstraints2=AllConstraints.u2;
        TXConstraints1=AllConstraints.Tx1;
        TXConstraints2=AllConstraints.Tx2;
        TUConstraints1=AllConstraints.Tu1;
        TUConstraints2=AllConstraints.Tu2;
        Ax1=AllConstraints.Ax1;
        Ax2=AllConstraints.Ax2;
        Cu1=AllConstraints.Cu1;
        Cu2=AllConstraints.Cu2;

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

        DotLyapunov=(Bbar(NoS * Prediction_Horizion - (NoS-1):NoS * Prediction_Horizion,:)')*psi*(errTX);

        den=sum([(hat_lambda_omega_x1 .* (-beta)  ./ (-beta * TXConstraints1+ 1)); (hat_lambda_omega_x2 .* (-beta)  ./ (-beta * TXConstraints2+ 1));...
            (hat_lambda_omega_u1 .* (-beta)  ./ (-beta * TUConstraints1+ 1));(hat_lambda_omega_u2 .* (-beta)  ./ (-beta * TUConstraints2+ 1))]);


        % Gradient with respect to u
        grad_B_u = 2 * QuBar * errU  + 2 * Bbar' *QxBar * errX +2 * Bbar(NoS * Prediction_Horizion - (NoS-1):NoS * Prediction_Horizion,:)' *Qn * errTX;
        grad_B_u = grad_B_u...
            - Bbar'*(hat_lambda_x1 .* (-beta) .*Ax1 ./ (-beta * XConstraints1+ 1))...
            - Bbar'*(hat_lambda_x2 .* (-beta) .*Ax2 ./ (-beta * XConstraints2 + 1))...
            - hat_lambda_u1 .* (-beta) .* Cu1 ./ (-beta * UConstraints1 + 1)...
            - hat_lambda_u2 .* (-beta) .* Cu2 ./ (-beta * UConstraints2 + 1)...
            - DotLyapunov.*den;

        % TXConstraints1 = M1*theta-0.98*Xconstraint+1/beta;
        % TXConstraints2 = -M1*theta-0.98*Xconstraint+1/beta;
        % TUConstraints1 = M2*theta-0.98*Uconstraint+1/beta;
        % TUConstraints2 =- M2*theta-0.98*Uconstraint+1/beta;
    end