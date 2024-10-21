% ship parameters 
m = 17.0677e6;          % mass (kg)
Iz = 2.1732e10;         % yaw moment of inertia (kg m^2)
xg = -3.7;              % CG x-ccordinate (m)
L = 161;                % length (m)
B = 21.8;               % beam (m)
T = 8.9;                % draft (m)
KT = 0.7;               % propeller coefficient (-)
Dia = 3.3;              % propeller diameter (m)
rho = 1025;             % density of water (m/s^3)

% rudder limitations
delta_max  = 40 * pi/180;        % max rudder angle      (rad)
Ddelta_max = 5  * pi/180;        % max rudder derivative (rad/s)

% added mass matrix
Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;
MA = -[ Xudot 0 0; 0 Yvdot Yrdot; 0 Nvdot Nrdot ];

% rigid-body mass matrix
MRB = [ m 0    0 
        0 m    m*xg
        0 m*xg Iz ];
Minv = invQR(MRB + MA);

% rudder coefficients
b = 2;
AR = 8;
CB = 0.8;

lambda = b^2 / AR;
tR = 0.45 - 0.28*CB;
CN = 6.13*lambda / (lambda + 2.25);
aH = 0.75;
xH = -0.4 * L;
xR = -0.5 * L;

% input matrix
t_thr = 0.05;                                        % thrust deduction number
X_delta2 = 0.5 * (1 - tR) * rho * AR * CN;           % rudder coefficients (Section 9.5)
Y_delta = 0.25 * (1 + aH) * rho * AR * CN; 
N_delta = 0.25 * (xR + aH*xH) * rho * AR * CN;   

Bi = @(u_r,delta) [ (1-t_thr)  -u_r^2 * X_delta2 * delta
                        0      -u_r^2 * Y_delta
                        0      -u_r^2 * N_delta            ];
    

% linear damping
T1 = 20; T2 = 20; T6 = 10;
Xu = -(m-Xudot)/T1;
Yv = -(m-Yvdot)/T2;
Nr = -(Iz-Nrdot)/T6;
D = -diag([Xu Yv Nr]);

% nonlinear surge damping
eps = 0.001;
CR = 0;
k = 0.1;
S = B*L + 2*T*(B+L);
v = 1e-6;

% nonlinear cross-flow drag
Cd_2d = Hoerner(B,T);
dx = L/10;
Ycf = 0; Ncf = 0;

u_d   = 7;            % desired surge speed (m/s)


Minv_sway_yaw = Minv(2:3,2:3);

CRB_lin = [0 m*u_d;
           0 -3.7*m*u_d];

CA_lin = [0 -Xudot*u_d;
         (Xudot-Yvdot)*u_d -Yrdot*u_d];

N = CRB_lin + CA_lin -diag([Yv Nr]);

b = 2*u_d*[-Y_delta; -N_delta];

ss_A = -Minv_sway_yaw*N;
ss_B = Minv_sway_yaw*b;

[num,den] = ss2tf(ss_A,ss_B,[0 1],0);

%%

first_tf = tf(num,den)

new_tf = tf(num/den(3),den/den(3))

T_3 = 0.1053/0.007493


