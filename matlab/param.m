clear; clc; close all;

P.Ts = 0.01;

P.takeoff_time = 5;
P.height = -20; %-1.5;     % desired height
P.one_lap = 10;   % expected time for one lap
P.omega = 2*pi/P.one_lap;
P.radius = 5;
P.g = 9.81;     % gravity

% physical parameters of airframe
P.gravity = 9.81;
P.mass    = 4.34;   % [kg]
P.Jxx     = 0.0820; % [kg-m2]
P.Jyy     = 0.0845; % [kg-m2]
P.Jzz     = 0.1377; % [kg-m2]

% The dist from CoM to the center of ea. rotor in the b1-b2 plane
P.d  = 0.315; % [m]

% first cut at initial conditions
P.p0 = [0 0 0];
P.v0 = [0 0 -0.2];
P.R0 = expm(skew(deg2rad([0 0 0])));
P.Omega0 = deg2rad([0 0 0]);

% sketch parameters
P.nRotors = 4;

% time constant for dirty derivative filter
P.tau = 0.15;