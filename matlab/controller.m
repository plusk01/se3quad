function out = controller(u,P)

% process inputs
xd    = u(1:3);
b1d   = u(4:6);
% current state
x     = u(7:9);
v     = u(10:12);
R     = reshape(u(13:21),3,3);
Omega = u(22:24);
t     = u(end);

persistent dx1dt;
persistent dx2dt;
persistent dx3dt;
persistent dx4dt;
persistent db1dt;
persistent db2dt;
persistent dv1dt;
persistent dv2dt;

if t == 0
    dx1dt = DirtyDerivative(1, P.tau, P.Ts);
    dx2dt = DirtyDerivative(2, P.tau*10, P.Ts);
    dx3dt = DirtyDerivative(3, P.tau*10, P.Ts);
    dx4dt = DirtyDerivative(4, P.tau*10, P.Ts);
    
    db1dt = DirtyDerivative(1, P.tau, P.Ts);
    db2dt = DirtyDerivative(2, P.tau*10, P.Ts);
    
    dv1dt = DirtyDerivative(1, P.tau, P.Ts);
    dv2dt = DirtyDerivative(2, P.tau*10, P.Ts);
end

% numerical derivatives of desired position, xd
% [xd_1dot, xd_2dot, xd_3dot, xd_4dot] = dxdt(xd, t==0, P.tau, P.Ts);
xd_1dot = dx1dt.calculate(xd);
xd_2dot = dx2dt.calculate(xd_1dot);
xd_3dot = dx3dt.calculate(xd_2dot);
xd_4dot = dx4dt.calculate(xd_3dot);

% numerical derivatives of desired body-1 axis, b1d
b1d_1dot = db1dt.calculate(b1d);
b1d_2dot = db2dt.calculate(b1d_1dot);

% numerical derivaties of current state velocity, v
v_1dot = dv1dt.calculate(v);
v_2dot = dv2dt.calculate(v_1dot);

% calculate errors, eq 17-18
ex = x - xd;
ev = v - xd_1dot;
ea = v_1dot - xd_2dot;
ej = v_2dot - xd_3dot;

% inertial frame 3-axis
e3 = [0;0;1];

% thrust magnitude control, eq 19
A = -P.kx*ex - P.kv*ev - P.mass*P.gravity*e3 + P.mass*xd_2dot;
f = dot(-A, R*e3);


% normalized feedback function, eq 23
b3c = -A/norm(A);

% find a b1c that is orthogonal to b3c
% https://www.mathworks.com/matlabcentral/answers/72631#answer_82726
% N = null(b3c(:).');
% b1c = N(:,1);

% Construct b1c, eq 38
C = cross(b3c, b1d); % arXiv:1003.2005v3, eq ~97
b1c = -(1/norm(C))*cross(b3c, C);

b2c = C/norm(C);

% computed attitude, eq 22
Rc = [b1c b2c b3c];

% time derivatives of body axes -- arXiv:1003.2005v3, Appendix F
A_1dot   = -P.kx*ev - P.kv*ea + P.mass*xd_3dot;
b3c_1dot = -A_1dot/norm(A) + (dot(A,A_1dot)/norm(A)^3)*A;
C_1dot   = cross(b3c_1dot, b1d) + cross(b3c, b1d_1dot);
b2c_1dot = C/norm(C) - (dot(C,C_1dot)/norm(C)^3)*C;
% b2c_1dot = -1*b2c_1dot;
b1c_1dot = cross(b2c_1dot, b3c) + cross(b2c, b3c_1dot);

% second time derivatives of body axes
A_2dot   = -P.kx*ea - P.kv*ej + P.mass*xd_4dot;
b3c_2dot = -A_2dot/norm(A) + (2/norm(A)^3)*dot(A,A_1dot)*A_1dot ...
         + ((norm(A_1dot)^2 + dot(A,A_2dot))/norm(A)^3)*A       ...
         - (3/norm(A)^5)*(dot(A,A_1dot)^2)*A;
C_2dot   = cross(b3c_2dot, b1d) + cross(b3c, b1d_2dot)          ...
         + 2*cross(b3c_1dot, b1d_1dot);
b2c_2dot = C_2dot/norm(C) - (2/norm(C)^3)*dot(C,C_1dot)*C_1dot  ...
         - ((norm(C_2dot)^2 + dot(C,C_2dot))/norm(C)^3)*C       ...
         + (3/norm(C)^5)*(dot(C,C_1dot)^2)*C;
% b2c_2dot = -1*b2c_2dot;
b1c_2dot = cross(b2c_2dot, b3c) + cross(b2c, b3c_2dot)          ...
         + 2*cross(b2c_1dot, b3c_1dot);

% Extract calculated angular velocities and their time-derivatives
Rc_1dot     = [b1c_1dot b2c_1dot b3c_1dot];
Rc_2dot     = [b1c_2dot b2c_2dot b3c_2dot];
Omegac      = vee(Rc.'*Rc_1dot);
Omegac_1dot = vee(Rc.'*Rc_2dot - hat(Omegac)*hat(Omegac));

% inertia matrix
J = diag([P.Jxx P.Jyy P.Jzz]);

% more error, eq 21
eR     = (1/2)*vee(Rc.'*R - R.'*Rc);
eOmega = Omega - R.'*Rc*Omegac;

% moment vector control
M = -P.kR*eR - P.kOmega*eOmega + cross(Omega, J*Omega) ...
  - J*(hat(Omega)*R.'*Rc*Omegac - R.'*Rc*Omegac_1dot);

% calculate SO(3) error function, Psi
Psi = (1/2)*trace(eye(3) - Rc.'*R);

deltaF = P.Mix*[f;M];

out = [f;M;xd;xd_1dot;Omegac;Psi;deltaF];
end