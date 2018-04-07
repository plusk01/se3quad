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
P.mass    = 3.81;
P.Jxx     = 0.060224;
P.Jyy     = 0.122198;
P.Jzz     = 0.132166;

P.L  = 0.25;
P.k1 = 1; %2.98*10e-6;
P.k2 = 1; %2.98*10e-6;
P.mu = 1;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = 0;  % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

% sketch parameters
P.nRotors = 4;

% time constant for dirty derivative filter
P.tau = 0.15;