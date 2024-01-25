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

Tf = 30;

% Initial conditons
x0 = zeros(12,1);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% Setup reference function
ref = @(t_ , x_ ) ref_TVC(t_);

% Simulate
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);

% Visualize
rocket.anim_rate = 10; 
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; 
