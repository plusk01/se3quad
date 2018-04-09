function [sys,x0,str,ts,simStateCompliance] = quad_dynamics(t,x,u,flag,P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 18;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 18;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [P.p0 P.v0 P.R0(:)' P.Omega0];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%

% Double Checked March 4
function sys=mdlDerivatives(t,x,uu,P)
% Quadrotor equations of motion taken from
% Lee et al, Control of Complex Maneuvers for a Quadrotor UAV using
% Geometric Methods on SE(3), 2011.

    % position in the inertial frame
    p       = x(1:3);
    % velocity in the inertial frame
    v       = x(4:6);
    % orientation of body {b1,b2,b3} wrt inertial frame {e1,e2,e3}
    R       = reshape(x(7:15), 3, 3);
    % angular velocities in the body-fixed frame
    Omega   = x(16:18);
    
    f       = uu(1);
    M       = uu(2:4);
    
    % state derivative vector
    xdot = zeros(size(x));
    
    % inertia matrix
    J = diag([P.Jxx P.Jyy P.Jzz]);
       
    % Third axis of inertial frame
    e3 = [0;0;1];
    
        
    % eq 2
    xdot(1:3) = v;
    
    % eq 3
    xdot(4:6) = P.gravity*e3 - (1/P.mass)*f*R*e3;
    
    % eq 4
    Rdot = R*hat(Omega);
    xdot(7:15) = Rdot(:);
    
    % eq 5
    xdot(16:18) = J\(M - cross(Omega,J*Omega));
    
    
    % output
    sys = xdot;
% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
%     pn      = x(1);
%     pe      = x(2);
%     pd      = x(3);
%     u       = x(4);
%     v       = x(5);
%     w       = x(6);
%     phi     = x(7);
%     theta   = x(8);
%     psi     = x(9);
%     p       = x(10);
%     q       = x(11);
%     r       = x(12);
    
sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
