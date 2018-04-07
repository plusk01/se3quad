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

% numerical derivatives of desired position, pd
[xd_dot, xd_2dot, xd_3dot, xd_4dot] = dxdt(xd, t==0, P.tau, P.Ts);

% calculate errors, eq 17-18
ex = x - xd;
ev = v - xd_dot;

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

b2c = -C/norm(C);

% computed attitude, eq 22
Rc = [b1c cross(b3c,b1c) b3c];

ea = ev;
Adot = -P.kx*ev - P.kv*ea + P.mass*xd_3dot;

b3c_dot = -Adot/norm(Adot) + (dot(A,Adot)/norm(Adot)^3)*A;
% b2c_dot = 


% f = 42.43;
M = [0;0;0];

out = [f;M;xd;xd_dot];

end