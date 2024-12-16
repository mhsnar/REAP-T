function sigma = CostF(r,u_des,x0,inc,Abar,Bbar,M1Bar,M2Bar,hat_u,QxBar,QuBar,Qn,Qv,Prediction_Horizion,NoS,DeltaT,M1,M2,N,u_app,theta)

        xbar=M1*theta;
        ubar=M2*theta;
        V=N*theta;
        % if inc==1
        %     x0=x(:,1);
        % elseif inc>1
        %     x0=[x(1,inc);(x(1,inc)-x(1,inc-1))/DeltaT;x(3,inc);(x(3,inc)-x(3,inc-1))/DeltaT;x(5,inc);(x(5,inc)-x(5,inc-1))/DeltaT;u_app(:,inc-1)];
        % end
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
    end
