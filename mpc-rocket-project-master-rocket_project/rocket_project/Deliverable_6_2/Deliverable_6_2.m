addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/40;

rocket = Rocket(Ts);

H = 0.1;
x0 = zeros(12,1);
ref = [0.5, 0, 1, deg2rad(65)]';

delay = 5;
expected_delay = 4;
nmpc = NmpcControl(rocket, H, expected_delay);
Tf = 2.5;
rocket.mass = 1.75;
rocket.delay = delay;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

rocket.anim_rate = 2;
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'NMPC simulation partially delay-compensating controller';

delay = 5;
expected_delay = 5;
nmpc = NmpcControl(rocket, H, expected_delay);
Tf = 2.5;
rocket.mass = 1.75;
rocket.delay = delay;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

rocket.anim_rate = 2;
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'NMPC simulation fully delay-compensating controller';