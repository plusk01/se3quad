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
x     = u(1:3);
v     = u(4:6);
R     = reshape(u(7:15),3,3);
Omega = u(16:18);
% desired states
xd    = u(19:21);
vd    = u(22:24);
% time
t     = u(end);

persistent hx1; persistent hxd1;
persistent hx2; persistent hxd2;
persistent hx3; persistent hxd3;
persistent hv1; persistent hvd1;
persistent hv2; persistent hvd2;
persistent hv3; persistent hvd3;

if t==0
    figure(2), clf;
    
    subplot(321);
    hx1  = updatestate(t,x(1),[]); hold on;
    hxd1 = updatestate(t,xd(1),[]);
    ylabel('x'); grid on;
    
    subplot(323);
    hx2  = updatestate(t,x(2),[]); hold on;
    hxd2 = updatestate(t,xd(2),[]);
    ylabel('y'); grid on;
    
    subplot(325);
    hx3  = updatestate(t,x(3),[]); hold on;
    hxd3 = updatestate(t,xd(3),[]);
    ylabel('z'); grid on;
    
    subplot(322);
    hv1  = updatestate(t,v(1),[]); hold on;
    hvd1 = updatestate(t,vd(1),[]);
    ylabel('v_x'); grid on;
    
    subplot(324);
    hv2  = updatestate(t,v(2),[]); hold on;
    hvd2 = updatestate(t,vd(2),[]);
    ylabel('v_y'); grid on;
    
    subplot(326);
    hv3  = updatestate(t,v(3),[]); hold on;
    hvd3 = updatestate(t,vd(3),[]);
    ylabel('v_z'); grid on;
else
    updatestate(t,x(1),hx1); updatestate(t,xd(1),hxd1);
    updatestate(t,x(2),hx2); updatestate(t,xd(2),hxd2);
    updatestate(t,x(3),hx3); updatestate(t,xd(3),hxd3);
    updatestate(t,v(1),hv1); updatestate(t,vd(1),hvd1);
    updatestate(t,v(2),hv2); updatestate(t,vd(2),hvd2);
    updatestate(t,v(3),hv3); updatestate(t,vd(3),hvd3);
end




end


function handle = updatestate(t, x, handle)
if isempty(handle)
	handle = plot(t, x);
else
    XX = get(handle,'XData'); YY = get(handle,'YData');
    set(handle,'XData',[XX t],'YData',[YY x]);
    drawnow
end
end