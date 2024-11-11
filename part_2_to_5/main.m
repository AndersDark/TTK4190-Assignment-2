% Project in TTK4190 Guidance, Navigation and Control of Vehicles 
%
% Author:           My name
% Study program:    My study program

% Add folder for 3-D visualization files
addpath(genpath('flypath3d_v2'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
clear WP_selector;
T_final = 1000;	        % Final simulation time (s)
h = 0.1;                % Sampling time (s)

psi_ref = 0;  % desired yaw angle (rad)
U_ref   = 9;            % desired surge speed (m/s)

% initial states
eta_0 = [0 0 deg2rad(-110)]';
nu_0  = [0 0 0]';
delta_0 = 0;
n_0 = 0;
xd = [0 0 0]';
e_int = 0;
Qm_0 = 0;
x = [nu_0' eta_0' delta_0 n_0 Qm_0]'; % The state vector can be extended with addional states here


% Kalman filter
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

x_prd = [0;0;0];
P_prd = 100*eye(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = 0:h:T_final;                % Time vector
nTimeSteps = length(t);         % Number of time steps

simdata = zeros(nTimeSteps, 15); % Pre-allocate matrix for efficiency
wait_bar = waitbar(0, 'Starting');

for i = 1:nTimeSteps
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 1a) Add current here 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    V_c = 1;
    beta_vc = deg2rad(45);
    psi = x(6);
    uc = V_c*cos(beta_vc-psi);
    vc = V_c*sin(beta_vc-psi);
    nu_c = [ uc vc 0 ]';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 1c) Add wind here 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    V_w = 10;
    beta_vw = deg2rad(135);
    rho_a = 1247;
    c_y = 0.95;
    c_n = 0.15;
    L_oa = 161; % boat length = 161m
    A_lw = 10*L_oa;

    gamma_w = psi-beta_vw-pi;

    C_Y = c_y*sin(gamma_w);
    C_N = c_n*sin(2*gamma_w);

    Ywind = C_Y*A_lw;
    Nwind = C_N*A_lw*L_oa;

    % Vessel speed
    U = sqrt(x(1)^2+x(2)^2);
    % Relative wind speed
    V_rw = U - V_w;

    tau_wind = 0;%1/2*rho_a*V_rw*[0 Ywind Nwind]';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 5, 2c)
    % Implement Kalman Filter
    %
    psi_meas = psi + normrnd(0,deg2rad(0.5));
    r_meas = x(3) + normrnd(0,deg2rad(0.1));
    rudder_meas = x(7);
    [x_pst,P_pst,x_prd,P_prd] = kf_iteration(x_prd,P_prd,Ad.A,Ad.B,Ad.C,psi_meas,rudder_meas);
    psi_est = x_pst(1);
    r_est = x_pst(1);
    bias_est = x_pst(1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 2d) Add a reference model here 
    % Define it as a function
    % check eq. (15.143) in (Fossen, 2021) for help
    %
    % The result should look like this:
    % xd_dot = ref_model(xd,psi_ref(i));
    % psi_d = xd(1);
    % r_d = xd(2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Guidance law
    [xk1,yk1,xk,yk,last_waypoint_reached] = WP_selector(x(4),x(5));
    [e_y, pi_p] = cross_track_error(xk1,yk1,xk,yk,x(4),x(5));
    % chi_d = LOS_guidance(e_y, pi_p);
    chi_d = ILOS_guidance(pi_p, e_y, h);
    psi_ref = chi_d;


    % Crab angle compensation
    % crab_angle = atan2(x(2),x(1));
    % psi_ref = psi_ref - crab_angle;

    % Refrence model
    xd_dot = ref_model(xd,psi_ref);
    xd = xd + xd_dot * h;

    psi_d = xd(1);
    r_d = xd(2);

    u_d = U_ref;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 2d) Add the heading controller here 
    % Define it as a function
    %
    % The result should look like this:
    % delta_c = PID_heading(e_psi,e_r,e_int);

    e_psi = ssa(psi_est-psi_d);
    e_r   = r_est - r_d;
    e_int = 0;%e_int + e_psi * h;
    delta_c = PID_heading(e_psi,e_r,e_int); % rudder angle command (rad)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 3, 1e) Add open loop speed control here
    % Define it as a function
    %
    % The result should look like this:
    % n_c = open_loop_speed_control(U_ref);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    n_c = open_loop_speed_control(U_ref);                   % propeller speed [radians per second (rps)]

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 3, 1f) Replace the open loop speed controller, 
    % with a closed loop speed controller here 
    % Define it as a function
    %
    % The result should look like this:
    % n_c = closed_loop_speed_control(u_d,e_u,e_int_u);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % ship dynamics
    u = [delta_c n_c]';
    [xdot,u] = ship(x,u,nu_c,tau_wind);
    
    % store simulation data in a table (for testing)
    simdata(i,:) = [x(1:3)' x(4:6)' x(7) x(8) u(1) u(2) u_d psi_d r_d uc vc];     
 
    % Euler integration
    % x = euler2(xdot,x,h); 
    % Runge Kutta 4 integration
    x = rk4(@ship,h,x,u,nu_c,tau_wind);

    waitbar(i/nTimeSteps, wait_bar, sprintf('Progress: %d %%', floor(i/nTimeSteps*100)));

    if last_waypoint_reached
        break;
    end
end
close(wait_bar);
simdata = simdata(1:i,:);
t = t(1:i);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u           = simdata(:,1);                 % m/s
v           = simdata(:,2);                 % m/s
r           = simdata(:,3);                 % rad/s
r_deg       = (180/pi) * r;                 % deg/s
x           = simdata(:,4);                 % m
y           = simdata(:,5);                 % m
psi         = simdata(:,6);                 % rad
psi_deg     = (180/pi) * psi;               % deg
delta_deg   = (180/pi) * simdata(:,7);      % deg
n           = (30/pi) * simdata(:,8);       % rpm
delta_c_deg = (180/pi) * simdata(:,9);      % deg
n_c         = (30/pi) * simdata(:,10);      % rpm
u_d         = simdata(:,11);                % m/s
psi_d       = simdata(:,12);                % rad
psi_d_deg   = (180/pi) * psi_d;             % deg
r_d         =  simdata(:,13);               % rad/s
r_d_deg     = (180/pi) * r_d;               % deg/s
%% custom variables
uc = simdata(:,14);
vc = simdata(:,15);
crab_angle = (180/pi) * atan2(v,u);
sideslip_angle = (180/pi) * atan2(v-vc,u-uc);

%% Ploting
pathplotter(x,y)

figure(4)
figure(gcf)
hold on;
plot(t,sideslip_angle,'LineWidth',2')
plot(t,crab_angle,'LineStyle','--','LineWidth',2)
title('Crab angle and Sideslip angle');
xlabel('Time (s)');  ylabel('Angle (deg)'); 
legend('Sideslip angle','Crab angle')
grid on;

figure(3)
figure(gcf)
hold on;
plot(t,psi_d_deg,'linewidth',2); % course angle desired
plot(t,psi_deg+crab_angle,'LineWidth',2); %course angle
plot(t,psi_deg,'linewidth',2); % Heading
xlabel('Time (s)');  ylabel('Angle (deg)'); 
legend('desired course angle','course angle','Heading angle')
grid on;


% figure(3)
% figure(gcf)
% subplot(311)
% plot(y,x,'linewidth',2); axis('equal')
% title('North-East positions'); xlabel('(m)'); ylabel('(m)'); 
% subplot(312)
% plot(t,psi_deg,t,psi_d_deg,'linewidth',2);
% title('Actual and desired yaw angle'); xlabel('Time (s)');  ylabel('Angle (deg)'); 
% legend('actual yaw','desired yaw')
% subplot(313)
% plot(t,r_deg,t,r_d_deg,'linewidth',2);
% title('Actual and desired yaw rates'); xlabel('Time (s)');  ylabel('Angle rate (deg/s)'); 
% legend('actual yaw rate','desired yaw rate')

% figure(2)
% figure(gcf)
% subplot(211)
% plot(t,u,t,u_d,'linewidth',2);
% title('Actual and desired surge velocity'); xlabel('Time (s)'); ylabel('Velocity (m/s)');
% legend('actual surge','desired surge')
% subplot(212)
% plot(t,n,t,n_c,'linewidth',2);
% title('Actual and commanded propeller speed'); xlabel('Time (s)'); ylabel('Motor speed (RPM)');
% legend('actual RPM','commanded RPM')
% subplot(313)
% plot(t,delta_deg,t,delta_c_deg,'linewidth',2);
% title('Actual and commanded rudder angle'); xlabel('Time (s)'); ylabel('Angle (deg)');
% legend('actual rudder angle','commanded rudder angle')

%% Create objects for 3-D visualization 
% Since we only simulate 3-DOF we need to construct zero arrays for the 
% excluded dimensions, including height, roll and pitch
% z = zeros(length(x),1);
% phi = zeros(length(psi),1);
% theta = zeros(length(psi),1);

% create object 1: ship (ship1.mat)
% new_object('flypath3d_v2/ship1.mat',[x,y,z,phi,theta,psi],...
% 'model','royalNavy2.mat','scale',(max(max(abs(x)),max(abs(y)))/1000),...
% 'edge',[0 0 0],'face',[0 0 0],'alpha',1,...
% 'path','on','pathcolor',[.89 .0 .27],'pathwidth',2);

% Plot trajectories 
% flypath('flypath3d_v2/ship1.mat',...
% 'animate','on','step',500,...
% 'axis','on','axiscolor',[0 0 0],'color',[1 1 1],...
% 'font','Georgia','fontsize',12,...
% 'view',[-25 35],'window',[900 900],...
% 'xlim', [min(y)-0.1*max(abs(y)),max(y)+0.1*max(abs(y))],... 
% 'ylim', [min(x)-0.1*max(abs(x)),max(x)+0.1*max(abs(x))], ...
% 'zlim', [-max(max(abs(x)),max(abs(y)))/100,max(max(abs(x)),max(abs(y)))/20]); 