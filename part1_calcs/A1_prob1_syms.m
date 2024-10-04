syms u v w p q r m R44 R55

L = 161;
B = 21.8;
H = 15.8;
% m = 17.0677e6;
I_z = 3.7544e+10;
R66 = sqrt(I_z/m);

r_bg = [-3.7, 0, H/2];


[M_RB_CO C_RB] = rbody(m,R44,R55,R66,[p; q; r],r_bg');