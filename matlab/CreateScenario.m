function scene = CreateScenario()
%CREATESCENARIO Ten heterogeneous CAVs in one jointly optimized cluster.
% State order: [x, y, v, a, phi, omega, theta, xc, yc].
p = struct('nodes',200,'max_iterations',6,'horizon',40, ...
    'smoothness',0.001,'penalty',200,'slack_weight',200,'tolerance',1e-6, ...
    'vmin',3,'vmax',10,'amax',2,'phimax',0.4,'omegamax',0.3, ...
    'attractive',0.001,'repulsive',0.6,'turning',0.6,'distance_scale',0.5, ...
    'box_step',0.25,'box_limit',5);
p.dt = p.horizon/(p.nodes-1);
scene.parameters = p;
scene.workspace = [-310,310,-310,310];
scene.obstacles = [-310,-24,-310,-22;24,310,-310,-25; ...
    -310,-27,24,310;26,310,27,310];
geometry = [5.73,1.71,1.90,3.50;5.30,1.50,1.60,3.20;4.90,1.30,1.50,3.00];
headings = [0,pi/2,pi,-pi/2];
for i = 1:10
    g = geometry(mod(i-1,3)+1,:);
    vehicle = struct('LW',g(1),'LF',g(2),'LR',g(3),'LB',g(4));
    vehicle.heading = headings(mod(i-1,4)+1);
    u = [cos(vehicle.heading);sin(vehicle.heading)];
    lateral = 12*[u(2);-u(1)];
    vehicle.initial = zeros(9,1);
    vehicle.initial(1:2) = -(30+11.5*(i-1))*u+lateral;
    vehicle.initial(3) = 5;
    vehicle.initial(7) = vehicle.heading;
    vehicle.exit = 40*u+lateral;
    vehicle.desired = 410*u+lateral;
    vehicle.exit_speed = 5;
    vehicle.offset = 0.5*(vehicle.LW-vehicle.LR+vehicle.LF);
    vehicle.radius = 0.5*hypot(vehicle.LW+vehicle.LF+vehicle.LR,vehicle.LB);
    vehicle.initial(8:9) = vehicle.initial(1:2)+vehicle.offset*u;
    scene.vehicles(i) = vehicle; %#ok<AGROW>
end
end
