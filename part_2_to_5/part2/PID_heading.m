function delta_c = PID_heading(e_psi,e_r,e_int)
w_b = 0.06;
zeta = 1;

w_n = 1/sqrt(1-2*zeta^2 + sqrt(4*zeta^4-4*zeta^2+2)) * w_b;

K = 0.0075;
T = 169.549;

m = T/K;
d = 1/K;
k = 0;

Kp = m*w_n^2-k;
Kd = 2*zeta*w_n*m-d;
Ki = w_n/10*Kp;

delta_c = -(Kp*e_psi + Kd*e_r + Ki*e_int);

end

