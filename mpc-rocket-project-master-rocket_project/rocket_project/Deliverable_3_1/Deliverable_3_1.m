addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable

close all
clear all
clc

Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controller
H = 10; % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Simulation :
Tf = 10; % seconds

x0 = [0; 0; 0; 3];
y0 = [0; 0; 0; 3];
z0 = [0; 3];
roll0 = [0; deg2rad(30)];

% Closed loop plots
%x_x
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub (T, X_sub, U_sub, sys_x, xs, us);

%x_y
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub (T, X_sub, U_sub, sys_y, xs, us);

%x_z
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub (T, X_sub, U_sub, sys_z, xs, us);

%x_roll
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub (T, X_sub, U_sub, sys_roll, xs, us);



% Open loop plots
%x_x
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x0);
%pad last input to get consistent size with time and state
U_opt(:, end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us);

%x_y
[u, T_opt, X_opt, U_opt] = mpc_y.get_u(y0);
%pad last input to get consistent size with time and state
U_opt(:, end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us);

%x_z
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(z0);
%pad last input to get consistent size with time and state
U_opt(:, end+1) = NaN;
% Account for linearization point
U_opt(:) = U_opt(:) + us(3);
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);

%x_roll
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll0);
%pad last input to get consistent size with time and state
U_opt(:, end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);
