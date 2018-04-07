function ss = hat(vec)
% function ss = hat(vec)
%   The hat function maps a vector in R^3 (or R^1) to its
%   skew symmetric matrix in R^(3x3) (or R^(2x2))

% https://github.com/justinthomas/MATLAB-tools/blob/master/hat.m

switch length(vec)
    
    case 3
        ss = [...
            0, -vec(3), vec(2);...
            vec(3), 0, -vec(1);...
            -vec(2), vec(1), 0];
        
    case 1
        ss = [...
            0, vec; ...
            -vec, 0];
end

end