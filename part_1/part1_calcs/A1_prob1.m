clear

L = 161;
B = 21.8;
H = 15.8;
m = 17.0677e6;
rho_b = m/(L*B*H);

I_zz = 1/12 * m*(L^2+B^2);

M_RB_CG = m*eye(3);

r_bg = [-3.7, 0, H/2];


[M_RB_CO C_RB] = rbody(m,L,B,H,[0;0;1],r_bg');