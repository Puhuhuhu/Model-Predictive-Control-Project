addpath(fullfile('..', 'src'));

%close all
%clear all
%clc

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

Tf = 10;

% Initial conditons
x0 = [0; 0; 0; 0];
y0 = [0; 0; 0; 0];
z0 = [0; 0];
roll0 = [0; 0];

% Tracked points
ref_x = -4;
ref_y = -4;
ref_z = -4;
ref_roll = deg2rad(35);


% Closed-loop plots
% x dimension
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, ref_x);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, ref_x);
% y dimension
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, ref_y);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, ref_y);
% z dimension
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, ref_z);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, ref_z);
% roll dimension
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, ref_roll);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, ref_roll);



% Open loop plots
%x_x
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x0, ref_x);
%pad last input to get consistent size with time and state
U_opt(:, end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us);
%x_y
[u, T_opt, X_opt, U_opt] = mpc_y.get_u(y0, ref_y);
%pad last input to get consistent size with time and state
U_opt(:, end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us);
%x_z
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(z0, ref_z);
%pad last input to get consistent size with time and state
U_opt(:, end+1) = NaN;
% Account for linearization point
U_opt(:) = U_opt(:) + us(3);
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);
%x_roll
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll0, ref_roll);
%pad last input to get consistent size with time and state
U_opt(:, end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);