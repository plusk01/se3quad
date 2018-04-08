function plotter(uu, P)
	try
		plotter_(uu, P);
	catch e
		msgString = getReport(e);
		fprintf(2,'%s\n', msgString);
		rethrow(e);
	end
end

function plotter_(u, P)

% current state
x      = u(1:3);
v      = u(4:6);
R      = reshape(u(7:15),3,3);
Omega  = u(16:18);
% desired states
xd     = u(19:21);
vd     = u(22:24);
Omegac = u(25:27);
% extras
Psi    = u(28);
deltaF = u(29:32);
% time
t     = u(end);

persistent hx1; persistent hxd1;
persistent hx2; persistent hxd2;
persistent hx3; persistent hxd3;
persistent hv1; persistent hvd1;
persistent hv2; persistent hvd2;
persistent hv3; persistent hvd3;

persistent hO1; persistent hOc1;
persistent hO2; persistent hOc2;
persistent hO3; persistent hOc3;
persistent hPsi;

persistent hf1;
persistent hf2;
persistent hf3;
persistent hf4;

if t==0
    figure(2), clf;
    suptitle('Translational States')
    
    subplot(321);
    hx1  = updatestate(t,x(1),[]); hold on;
    hxd1 = updatestate(t,xd(1),[],'--');
    ylabel('x'); grid on;
    
    subplot(323);
    hx2  = updatestate(t,x(2),[]); hold on;
    hxd2 = updatestate(t,xd(2),[],'--');
    ylabel('y'); grid on;
    
    subplot(325);
    hx3  = updatestate(t,x(3),[]); hold on;
    hxd3 = updatestate(t,xd(3),[],'--');
    ylabel('z'); grid on;
    
    subplot(322);
    hv1  = updatestate(t,v(1),[]); hold on;
    hvd1 = updatestate(t,vd(1),[],'--');
    ylabel('v_x'); grid on;
    
    subplot(324);
    hv2  = updatestate(t,v(2),[]); hold on;
    hvd2 = updatestate(t,vd(2),[],'--');
    ylabel('v_y'); grid on;
    
    subplot(326);
    hv3  = updatestate(t,v(3),[]); hold on;
    hvd3 = updatestate(t,vd(3),[],'--');
    ylabel('v_z'); grid on;
    
    % =====================================================================
    
    figure(3), clf;
    suptitle('Rotational States');
    
    subplot(321);
    hO1  = updatestate(t,Omega(1),[]); hold on;
    hOc1 = updatestate(t,Omegac(1),[],'--');
    ylabel('\Omega_x'); grid on;
    
    subplot(323);
    hO2  = updatestate(t,Omega(2),[]); hold on;
    hOc2 = updatestate(t,Omegac(2),[],'--');
    ylabel('\Omega_y'); grid on;
    
    subplot(325);
    hO3  = updatestate(t,Omega(3),[]); hold on;
    hOc3 = updatestate(t,Omegac(3),[],'--');
    ylabel('\Omega_z'); grid on;
    
    subplot(3,2,[2 4 6]);
    hPsi = updatestate(t,Psi,[]);
    ylabel('\Psi'); grid on;
    
    % =====================================================================
    
    figure(4), clf;
    suptitle('Actuators');
    
    subplot(411);
    hf1  = updatestate(t,deltaF(1),[]);
    ylabel('f_1'); grid on;
    
    subplot(412);
    hf2  = updatestate(t,deltaF(2),[]);
    ylabel('f_2'); grid on;
    
    subplot(413);
    hf3  = updatestate(t,deltaF(3),[]);
    ylabel('f_3'); grid on;
    
    subplot(414);
    hf4  = updatestate(t,deltaF(4),[]);
    ylabel('f_4'); grid on;
    
else
    updatestate(t,x(1),hx1); updatestate(t,xd(1),hxd1);
    updatestate(t,x(2),hx2); updatestate(t,xd(2),hxd2);
    updatestate(t,x(3),hx3); updatestate(t,xd(3),hxd3);
    updatestate(t,v(1),hv1); updatestate(t,vd(1),hvd1);
    updatestate(t,v(2),hv2); updatestate(t,vd(2),hvd2);
    updatestate(t,v(3),hv3); updatestate(t,vd(3),hvd3);
    
    % =====================================================================
    
    updatestate(t,Omega(1),hO1); updatestate(t,Omegac(1),hOc1);
    updatestate(t,Omega(2),hO2); updatestate(t,Omegac(2),hOc2);
    updatestate(t,Omega(3),hO3); updatestate(t,Omegac(3),hOc3);
    updatestate(t,Psi,hPsi);
    
    % =====================================================================
    
    updatestate(t,deltaF(1),hf1);
    updatestate(t,deltaF(2),hf2);
    updatestate(t,deltaF(3),hf3);
    updatestate(t,deltaF(4),hf4);
end
end


function handle = updatestate(t, x, handle, style)
if isempty(handle)
    if nargin<4, style = '-'; end
	handle = plot(t, x, style);
else
    XX = get(handle,'XData'); YY = get(handle,'YData');
    set(handle,'XData',[XX t],'YData',[YY x]);
    drawnow
end
end