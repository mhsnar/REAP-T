function phi = Phi(hatLambda,grad_B_lambda,hat_lambda)
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
        % Adjust gradients based on conditions for Phi(s)
        lambda = [hat_lambda_x1;hat_lambda_x2;hat_lambda_u1;hat_lambda_u2...
            ;hat_lambda_omega_x1;hat_lambda_omega_x2;hat_lambda_omega_u1;hat_lambda_omega_u2;hat_lambda_omega_x1_eps;hat_lambda_omega_x2_eps;hat_lambda_omega_u1_eps;hat_lambda_omega_u2_eps] ;

        for k=1:length(hat_lambda)
            if lambda(k) > 0  || (lambda(k)== 0  && grad_B_lambda(k)>= 0)
                phi(k) = 0; % Set to zero based on conditions
            else
                if k==126
                    sd=1;
                end
                phi(k) = -(grad_B_lambda(k));
            end
        end

        phi=phi';
    end