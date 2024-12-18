function TCon = TconUpdate_Lyap(hat_u,ds,Sigma,grad_B_u,Abar,Bbar,x0,psi,Ad,Prediction_Horizion,xbar,ubar,NoS,NoI,Xconstraint,Uconstraint,Uconstraint_down,Xconstraint_down,beta,K)
        hat_u0= hat_u- ds*Sigma*grad_B_u;
        x_prr =Abar*x0+Bbar*hat_u0 ;
        invpsi=inv(psi);
        err=x_prr(size(Ad,1)*Prediction_Horizion-(size(Ad,1)-1):size(Ad,1)*Prediction_Horizion)-xbar;
        LyapanovFF=(err)'*psi*(err);
        TX1=[];
        TX2=[];
        coeff=eye(NoS);
        beta=50;

        for incc=1:NoS
            TX1=[TX1;LyapanovFF-((xbar(incc)-Xconstraint(incc))^2/(coeff(incc,:)*invpsi*coeff(incc,:)'))+1/beta];
            a1=((xbar(incc)-Xconstraint(incc))^2/(coeff(incc,:)*invpsi*coeff(incc,:)'));
            a2=((-xbar(incc)+Xconstraint_down(incc))^2/(coeff(incc,:)*invpsi*coeff(incc,:)'));


            TX2=[TX2;LyapanovFF-((-xbar(incc)+Xconstraint_down(incc))^2/(coeff(incc,:)*invpsi*coeff(incc,:)'))+1/beta];
        end

        TU1=[];
        TU2=[];
        for incc=1:NoI
            b1=((ubar(incc)-Uconstraint(incc))^2/(K(incc,:)*invpsi*K(incc,:)'));
            b2=((-ubar(incc)+Uconstraint_down(incc))^2/(K(incc,:)*invpsi*K(incc,:)'));

            TU1 = [TU1;LyapanovFF-((ubar(incc)-Uconstraint(incc))^2/(K(incc,:)*invpsi*K(incc,:)'))+1/beta];
            TU2 = [TU2;LyapanovFF-((-ubar(incc)+Uconstraint_down(incc))^2/(K(incc,:)*invpsi*K(incc,:)'))+1/beta];
        end
        TCon=[TX1;TX2;TU1;TU2];
end




