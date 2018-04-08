function out = trajectory(u,P)

t = u(end);

xd = [0 0 0];
b1d = [1 0 0];

f = 0.5;
% xd = [0 0 sin(2*pi*f*t)];

out = [xd b1d];

end