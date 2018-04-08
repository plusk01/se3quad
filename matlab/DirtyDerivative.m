classdef DirtyDerivative < handle
    %DIRTYDERIVATIVE Provide nth-order derivatives of a signal
    %
    %   This class creates a filtered derivative based on a band-limited
    %   low-pass filter with transfer function:
    %
    %       P(s) = s/(tau + s)
    %
    %   This is done because a pure differentiator (D(s) = s) is not causal
    %
    %   sample Usage:
    %
    %       dxdt = DirtyDerivative(0.05, 0.01);
    %       xdot = dxdt.update(x)
    
    properties (Access = public)
        % dirty derivative time constant: higher => increased smoothing.
        tau = 0.05;
        
        % Sample rate. This implementation assumes constant step-size.
        Ts = 0.01;
        
        % constants related to digital derivative
        a1;
        a2;
        
        % the order of the derivative (how many samples are needed)
        order;
        
        % current value of the derivative
        dot;
        
        % internal memory for the lagged signal value
        x_d1;
        
        % current sample number
        it = 1;
    end
    
    methods
        function dxdt = DirtyDerivative(order, tau, Ts)
            dxdt.a1 = (2*tau-Ts)/(2*tau+Ts);
            dxdt.a2 = 2/(2*tau+Ts);
            dxdt.order = order;
        end
        
        function xdot = calculate(dxdt, x)
            % we operate on everything as a col vector
            x = x(:);
            
            % on the very first iteration, initialize
            % memory with vectors of the same size as signal
            if dxdt.it == 1
                dxdt.dot   = zeros(size(x));
                dxdt.x_d1  = zeros(size(x));
            end
            
            if dxdt.it > dxdt.order
                % Calculate the derivative
                dxdt.dot = dxdt.a1*dxdt.dot + dxdt.a2*(x - dxdt.x_d1);
            end
            
            % increment the sample iteration number
            dxdt.it = dxdt.it + 1;
            
            % store value for next time
            dxdt.x_d1 = x;
            
            % output - col vector
            xdot = dxdt.dot;
        end
    end
     
end

