function [x_pst,P_pst,x_prd,P_prd] = kf_iteration(x_prd,P_prd,Ad,Bd,Cd,psi_meas,rudder_meas)

persistent Q R
if(isempty(Q))
    Q = [10 0 0;...
         0 1 0;...
         0 0 0.1];

    R = 0.5^2;
end

K = P_prd*Cd' / (Cd*P_prd*Cd' + R);
x_pst = x_prd + K*(psi_meas-Cd*x_prd);
P_pst = (eye(3)-K*Cd)*P_prd*(eye(3)-K*Cd)' + K*R*K';
x_prd = Ad*x_pst + Bd*rudder_meas;
P_prd = Ad*P_pst*Ad' + Q;
end

