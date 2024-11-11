% T = −99.4713 , K = −0.0049 for Uref = 9 m/s.
T = -99.4713;
K = -0.0049;
U_ref = 9;

% Continuous time dynamics
A = [0 1 0; 0 -1/T -K/T; 0 0 0];
B = [0; K/T; 0];
C = [1 0 0];

sys = ss(A, B, C, 0);

% discretized dynamics
[Ad, Bd] = c2d(sys, 0.1, 'foh');

% Observability analysis
kf_obsv = obsv(Ad.A, C);
kf_rank_obsv = rank(kf_obsv);