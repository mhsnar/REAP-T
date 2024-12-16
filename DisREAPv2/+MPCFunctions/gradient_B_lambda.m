 function grad_B_lambda = gradient_B_lambda( AllConstraints, beta)
        XConstraints1=AllConstraints.x1;
        XConstraints2=AllConstraints.x2;
        UConstraints1=AllConstraints.u1;
        UConstraints2=AllConstraints.u2;
        TXConstraints1=AllConstraints.Tx1;
        TXConstraints2=AllConstraints.Tx2;
        TUConstraints1=AllConstraints.Tu1;
        TUConstraints2=AllConstraints.Tu2;

        TXConstraints1_eps=AllConstraints.Tx1_eps;
        TXConstraints2_eps=AllConstraints.Tx2_eps;
        TUConstraints1_eps=AllConstraints.Tu1_eps;
        TUConstraints2_eps=AllConstraints.Tu2_eps;






        grad_B_lambda =  [-log10(-beta * XConstraints1 + 1);-log10(-beta * XConstraints2 + 1);...
            -log10(-beta * UConstraints1 + 1); -log10(-beta * UConstraints2 + 1);...
            -log10(-beta * TXConstraints1 + 1);-log10(-beta * TXConstraints2 + 1);...
            -log10(-beta * TUConstraints1 + 1); -log10(-beta * TUConstraints2 + 1);...
            -log10(-beta * TXConstraints1_eps + 1);-log10(-beta * TXConstraints2_eps + 1);...
            -log10(-beta * TUConstraints1_eps + 1); -log10(-beta * TUConstraints2_eps + 1)];


        % grad_B_lambda(XConstraints1 > 1/beta) = -100;

    end