addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable
close all
clear all
clc

Ts = 1/20;
rocket = Rocket(Ts);

% Open loop plot
H = 10;
nmpc = NmpcControl(rocket, H);
x0 = zeros(12, 1);
ref = [1, 1, 1, deg2rad(15)]';
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);
ph.fig.Name = 'NMPC simulation Open loop 10 seconds';

% Closed loop plots
H = 0.1;
nmpc = NmpcControl(rocket, H);
Tf = 30;

% Sim with gamma ref = 15°
x0 = zeros(12, 1);
ref = @(t_, x_) ref_TVC(t_);
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 10;
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'NMPC simulation 15°';

% Sim with gamma ref = 50°
x0 = zeros(12, 1);
roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 10;
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'NMPC simulation 50°';