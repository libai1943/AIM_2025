function fleet = SimulateMVSFM(scene)
%SIMULATEMVSFM Guiding attraction and repulsion, Eqs. (21)-(25).
% Direct steering updates belong to the initializer. LIOM subsequently
% enforces the steering-rate limit and the full bicycle equations.
p = scene.parameters;
nv = numel(scene.vehicles);
fleet = zeros(9,p.nodes,nv);
for i = 1:nv
    fleet(:,1,i) = scene.vehicles(i).initial;
end
for k = 1:p.nodes-1
    next = zeros(9,nv);
    for i = 1:nv
        vehicle = scene.vehicles(i);
        q = fleet(:,k,i);
        c = q(1:2)+vehicle.offset*[cos(q(7));sin(q(7))];
        u = [cos(vehicle.heading);sin(vehicle.heading)];
        along = (q(1:2)-vehicle.initial(1:2))'*u;
        guide = vehicle.initial(1:2)+2*(floor(along/2)+2)*u;
        direction = guide-q(1:2);
        force = p.attractive*direction/max(1e-12,norm(direction));
        for j = 1:nv
            if i == j, continue; end
            other = fleet(:,k,j);
            other_center = other(1:2)+scene.vehicles(j).offset* ...
                [cos(other(7));sin(other(7))];
            force = force+Repulsion(c-other_center, ...
                vehicle.radius+scene.vehicles(j).radius,p);
        end
        for b = scene.obstacles'
            nearest = min(max(c,b([1,3])),b([2,4]));
            force = force+Repulsion(c-nearest,vehicle.radius,p);
        end
        angle = atan2(force(2),force(1))-q(7);
        q(5) = Clamp(p.turning*atan2(sin(angle),cos(angle)),-p.phimax,p.phimax);
        q(4) = Clamp(force'*[cos(q(7));sin(q(7))],-p.amax,p.amax);
        q(4) = Clamp(q(4),(p.vmin-q(3))/p.dt,(p.vmax-q(3))/p.dt);
        if k == 1
            q(4:5) = vehicle.initial(4:5);
        end
        fleet(:,k,i) = q;
        z = q;
        z(1:2) = q(1:2)+p.dt*q(3)*[cos(q(7));sin(q(7))];
        z(7) = q(7)+p.dt*q(3)*tan(q(5))/vehicle.LW;
        z(3) = q(3)+p.dt*q(4);
        z(8:9) = z(1:2)+vehicle.offset*[cos(z(7));sin(z(7))];
        next(:,i) = z;
    end
    fleet(:,k+1,:) = reshape(next,9,1,nv);
end
for i = 1:nv
    fleet(6,1:end-1,i) = Clamp(diff(fleet(5,:,i))/p.dt,-p.omegamax,p.omegamax);
    fleet(6,[1,end],i) = 0;
end
end

function force = Repulsion(direction,radius,p)
distance = max(1e-9,norm(direction));
magnitude = p.repulsive*exp(Clamp((radius-distance)/p.distance_scale,-700,30));
force = magnitude*direction/distance;
end

function value = Clamp(value,lower,upper)
value = min(max(value,lower),upper);
end
