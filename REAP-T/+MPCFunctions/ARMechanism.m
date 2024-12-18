function [hat_u,hatLambda] = ARMechanism(sigma_hat_u,sigma_hat_u0,hat_u0,hat_u,hatLambda0,hatLambda)
 if sigma_hat_u<=sigma_hat_u0
        hatLambda=hatLambda;
        hat_u=hat_u;
    else
        hatLambda=hatLambda0;
        hat_u=hat_u0;
    end
end