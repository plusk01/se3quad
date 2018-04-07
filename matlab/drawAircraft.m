function drawAircraft(uu,P)
    % position in the inertial frame
    x       = uu(1:3);
    % velocity in the inertial frame
    v       = uu(4:6);
    % orientation of body {b1,b2,b3} wrt inertial frame {e1,e2,e3}
    R       = reshape(uu(7:15), 3, 3);
    % angular velocities in the body-fixed frame
    Omega   = uu(16:18);
    % desired states
    xd      = uu(19:21);
    vd      = uu(22:24);
    % time
    t       = uu(end);
    
    % define persistent variables 
    persistent spacecraft_handle;
    persistent commanded_position_handle;
    persistent center;
    persistent V;
    persistent F;
    persistent patchcolors;
    view_range = 10;
    close_enough_tolerance = 0.9;
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        
        [V, F, patchcolors] = sketch_copter(P);
        
        
        figure(1), clf;
        grid on;
        hold on;
        spacecraft_handle = drawSpacecraftBody(V,F,patchcolors,x,R,[]);
%         commanded_position_handle = drawCommandedPosition(x_c,y_c,z_c,yaw_c,...
%                                                []);
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the view angle for figure
        center = [x(1) x(2) -x(3)];
        axis([center(1)-view_range,center(1)+view_range, ...
              center(2)-view_range,center(2)+view_range, ...
              center(3)-view_range,center(3)+view_range]);
        hold on;
        
    % at every other time step, redraw base and rod
    else
        pos = [x(1) x(2) -x(3)];
        close_enough_tolerance*[view_range;view_range;view_range] - abs(pos - center);
        if (min(close_enough_tolerance*[view_range;view_range;view_range] - abs(pos-center)) < 0)
            center = pos;
            axis([center(1)-view_range,center(1)+view_range, ...
              center(2)-view_range,center(2)+view_range, ...
              center(3)-view_range,center(3)+view_range]);
        end
        
        drawSpacecraftBody(V,F,patchcolors,x,R,spacecraft_handle);
%         drawCommandedPosition(x_c,y_c,z_c,yaw_c,...
%                            commanded_position_handle);
    end
end

  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawSpacecraftBody(V,F,patchcolors,p,R,handle)
%   V = rotate(V', phi, theta, psi)';  % rotate spacecraft
  V = R*V';
  V = translate(V, p(1), p(2), p(3))';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  RR = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*RR;
  
  if isempty(handle)
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat');
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
end

function handle = drawCommandedPosition(x_c, y_c, z_c, yaw_c,...
                                     handle)
  V = translate([0; 0; 0], x_c, y_c, z_c)';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;
  
  if isempty(handle)
  handle = plot3(V(1),V(2),V(3),'*','markersize',10);
  else
    set(handle,'xdata',V(1),'ydata',V(2),'zdata',V(3));
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi)
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_yaw*R_pitch*R_roll;
  % rotate vertices
  XYZ = R*XYZ;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)
  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

function [V, F, colors] = sketch_copter(P)

%Draw Hex
% parameters for drawing aircraft
  % scale size
  scale = 5;
    
spoke.h = 0.02;
spoke.w = 0.02;
spoke.l = 0.25;

center.above = 0.1;
center.below = 0.1;
center.r = 0.1;

angle = 2*pi/P.nRotors;

% Define the vertices (physical location of vertices
  V_rotor = [...
	 0,       -spoke.w/2,  spoke.h/2; % inside 4 of spoke
     0,        spoke.w/2,  spoke.h/2; % inside 4 of spoke
     0,        spoke.w/2, -spoke.h/2; % inside 4 of spoke
     0,       -spoke.w/2, -spoke.h/2; % inside 4 of spoke
     spoke.l, -spoke.w/2,  spoke.h/2; % outside 4 of spoke
     spoke.l,  spoke.w/2,  spoke.h/2; % outside 4 of spoke
     spoke.l,  spoke.w/2, -spoke.h/2; % outside 4 of spoke
     spoke.l, -spoke.w/2, -spoke.h/2; % outside 4 of spoke
     0,        0,          -center.above; % tip
     cos(angle/2)*center.r,  sin(angle/2)*center.r,  center.below; % base1
     cos(angle/2)*center.r, -sin(angle/2)*center.r,  center.below; % base2
     0, 0,  center.below; % base_center
  ];

% define faces as a list of vertices numbered above
  F_rotor = [...
        1,2,3,4; % strut inside
        5,6,7,8; % strut outside
        1,2,6,5; % strut top
        2,3,7,6; % strut side
        3,4,8,7; % strut bottom
        4,1,5,8; % strut side
        9,10,11,11; % center face 
        12,10,11,11]; % center bottom 

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];
  mywhite = [1, 1, 1];

  colors_rotor = [...
     myred;...      
     myred;
     mygreen;...    
     myblue;...     
     myyellow;...   
     mycyan;... 
     mywhite;... 
     mywhite;... 
    ];

  V_rotor = scale*V_rotor;   % rescale vertices

  V = [];
  F = [];
  colors = [];
  for rotor = 0:P.nRotors-1
    angle = rotor*2*pi/P.nRotors;
    R = [cos(angle), sin(angle),0;
        -sin(angle), cos(angle),0;
         0,0,1];
    V = [V;V_rotor*R];
    F = [F;F_rotor+rotor*size(V_rotor,1)];
    colors = [colors;colors_rotor];
  end
  
  colors(3,:) = myred; 
  colors(4,:) = myred; 
  colors(5,:) = myred; 
  colors(6,:) = myred; 

end
  