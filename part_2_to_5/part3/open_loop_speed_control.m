function  n_c = open_loop_speed_control(U_ref)

t = 0.05; % thrust deduction parameter

T1 = 20;
m = 17.0677e6;
Xudot = -8.9830e5;
Xu = -(m-Xudot)/T1;

T_d = U_ref*Xu/(t-1);

Dia = 3.3;
rho = 1025;
KT = 0.6367;

n_c = sign(T_d)*sqrt(abs(T_d)/(rho*Dia^4*KT));

end

