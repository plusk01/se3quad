% Take the first and second numerical derivative of the commanded position
function [xdot, xddot, xdddot, xddddot] = dxdt(x, reset, tau, Ts)

persistent it; % how many times has this method been called?

persistent dot;
persistent x_d1;

persistent ddot;
persistent dot_d1;

persistent d3dot;
persistent ddot_d1;

persistent d4dot;
persistent d3dot_d1;

if reset == 1
    it = 0;
    
    dot = zeros(3,1);
    x_d1 = zeros(3,1);
    ddot = zeros(3,1);
    dot_d1 = zeros(3,1);
    d3dot = zeros(3,1);
    ddot_d1 = zeros(3,1);
    d4dot = zeros(3,1);
    d3dot_d1 = zeros(3,1);
end

% increment iteration counter
it = it + 1;

% constants related to digital derivative
d1 = (2*tau-Ts);
d2 = (2*tau+Ts);

if it > 1

    % first time derivative of x
    if tau == 0
        dot = (x - x_d1)/Ts;
    else
        dot = (d1/d2)*dot + (2/d2)*(x - x_d1);
    end
    
end
    
if it > 2

    % first time derivative of xdot
    if tau == 0
        ddot = (dot - dot_d1)/Ts;
    else
        ddot = (d1/d2)*ddot + (2/d2)*(dot - dot_d1);
    end
    
end

if it > 3

    % first time derivative of xddot
    if tau == 0
        d3dot = (ddot - ddot_d1)/Ts;
    else
        d3dot = (d1/d2)*d3dot + (2/d2)*(ddot - ddot_d1);
    end
    
end

if it > 4

    % first time derivative of xdddot
    if tau == 0
        d4dot = (d3dot - d3dot_d1)/Ts;
    else
        d4dot = (d1/d2)*d4dot + (2/d2)*(d3dot - d3dot_d1);
    end
    
end

% save for next iteration
x_d1 = x;
dot_d1 = dot;
ddot_d1 = ddot;
d3dot_d1 = d3dot;

% output
xdot = dot;
xddot = ddot;
xdddot = d3dot;
xddddot = d4dot;
end