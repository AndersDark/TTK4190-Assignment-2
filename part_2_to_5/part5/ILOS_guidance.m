function psi_d = ILOS_guidance(pi_p, y_e, dt)

persistent yi;
if(isempty(yi))
    yi = 0;
end

Delta = 161*10;
kappa = 0.1;
Kp = 1/Delta;
Ki = kappa*Kp;

dot_yi = Delta*y_e / (Delta^2 + (y_e + kappa*yi)^2);
psi_d = pi_p - atan(Kp*y_e + Ki*yi);

yi = yi + dot_yi*dt;
end

