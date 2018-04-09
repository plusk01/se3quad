clear; clc;

% Sample time of the controller
P.Ts = 0.01;

% Sample time of the state plotter
P.plotTs = 0.1;

% physical parameters of airframe
P.gravity = 9.81;   % [m/s/s]
P.mass    = 4.34;   % [kg]
P.Jxx     = 0.0820; % [kg-m2]
P.Jyy     = 0.0845; % [kg-m2]
P.Jzz     = 0.1377; % [kg-m2]

% The dist from CoM to the center of ea. rotor in the b1-b2 plane
P.d  = 0.315; % [m]

% actuator constant
P.c_tauf = 8.004e-3; % [m]

% Mixing matrix that relates thrust/moments to actuators
P.Mix = inv([1 1 1 1; 0 -P.d 0 P.d;...
         P.d 0 -P.d 0; -P.c_tauf P.c_tauf -P.c_tauf P.c_tauf]);

% first cut at initial conditions
P.p0 = [0 0 0];
P.v0 = [0 0 0];
P.R0 = expm(hat(deg2rad([178.2 0 0])));
P.Omega0 = deg2rad([0 0 0]);

% sketch parameters
P.nRotors = 4;

% time constant for dirty derivative filter
P.tau = 0.05;

% Control gains (taken from Lee2011, arXiv:1003.2005v4)
P.kx = 16*P.mass;
P.kv = 5.6*P.mass;
P.kR = 8.81;
P.kOmega = 2.54;

P.kx = 4*P.mass;
P.kv = 5.6*P.mass;
P.kR = 8.81;
P.kOmega = 2.54;