clc, clear;

% dirty-derivative time constant
tau = 0.0025;

% to use a simple finite difference, use tau = 0 (see algorithm)
% tau = 0;

Ts = 0.005;
t = 0:Ts:(2-Ts);

% sinusoid frequency
f = 10;

x = [sin(2*pi*f*t); zeros(2,length(t))];

% ======== Analytically Calculate Derivative ==============================

ydot = [2*pi*f*cos(2*pi*f*t); zeros(2,length(t))];
yddot = [-(2*pi*f)^2*sin(2*pi*f*t); zeros(2,length(t))];

% ======== Numerically Calculate Derivative (simple) ======================

zdot = [0 diff(x(1,:))]./[0 diff(t)];
zddot = [0 diff(zdot(1,:))]./[0 diff(t)];

% ======== Numerically Calculate Derivative (tustin) ======================

dxdt  = DirtyDerivative(1, tau, Ts);
dx2dt = DirtyDerivative(2, tau, Ts);

Xdot  = zeros(3,length(t));
Xddot = zeros(3,length(t));

for i = 1:length(t)  
    Xdot(:,i)  = dxdt.calculate(x(:,i));
    Xddot(:,i) = dx2dt.calculate(Xdot(:,i));
end

figure(1), clf;
subplot(311);
plot(t, x(1,:)); hold on;
ylabel('f(t)'); xlabel('t (s)'); grid on;

subplot(312);
plot(t, Xdot(1,:)); hold on;
plot(t, ydot(1,:));
plot(t, zdot(1,:));
ylabel('f''(t)'); xlabel('t (s)'); grid on;

subplot(313);
plot(t, Xddot(1,:)); hold on;
plot(t, yddot(1,:));
plot(t, zddot(1,:));
ylabel('f''''(t)'); xlabel('t (s)'); grid on;