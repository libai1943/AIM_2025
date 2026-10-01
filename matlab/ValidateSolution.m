function metrics = ValidateSolution(scene,fleet,require_feasible)
%VALIDATESOLUTION Recompute geometry, dynamics and all-pair discrete clearance.
% Does not reuse corridor boxes as a collision certificate.
p = scene.parameters;
nv = numel(scene.vehicles);
assert(all(isfinite(fleet(:))),'AIM:Finite','Nonfinite state encountered.');
centers = zeros(2,p.nodes,nv);
clearance = inf(1,p.nodes);
pair_clearance = clearance;
kinematic = 0; max_defect = 0; hard_violation = 0;
for i = 1:nv
    v = scene.vehicles(i); q = fleet(:,:,i); q0 = q(:,1:end-1);
    c = q(1:2,:)+v.offset*[cos(q(7,:));sin(q(7,:))];
    centers(:,:,i) = c;
    geometry = q(8:9,:)-c;
    dynamic = diff(q([1,2,3,5,7],:),1,2)/p.dt - ...
        [q0(3,:).*cos(q0(7,:));q0(3,:).*sin(q0(7,:));q0(4,:);q0(6,:); ...
        q0(3,:).*tan(q0(5,:))/v.LW];
    residual = [geometry(:);dynamic(:)];
    kinematic = kinematic+p.dt*sum(residual.^2);
    max_defect = max(max_defect,max(abs(residual)));
    w = scene.workspace;
    clearance = min([clearance;c(1,:)-w(1)-v.radius;w(2)-c(1,:)-v.radius; ...
        c(2,:)-w(3)-v.radius;w(4)-c(2,:)-v.radius],[],1);
    for b = scene.obstacles'
        dx = max(max(b(1)-c(1,:),c(1,:)-b(2)),0);
        dy = max(max(b(3)-c(2,:),c(2,:)-b(4)),0);
        clearance = min(clearance,hypot(dx,dy)-v.radius);
    end
    u = [cos(v.heading);sin(v.heading)];
    endpoint = q(1:2,end)-v.exit;
    hard_violation = max([hard_violation; p.vmin-q(3,:)';q(3,:)'-p.vmax; ...
        abs(q(4,:)')-p.amax;abs(q(5,:)')-p.phimax;abs(q(6,:)')-p.omegamax; ...
        abs(q(1:7,1)-v.initial(1:7)); ...
        abs(q(3:7,end)-[v.exit_speed;0;0;0;v.heading]); ...
        -u'*endpoint;abs([-u(2),u(1)]*endpoint)]);
end
for i = 1:nv
    for j = i+1:nv
        d = centers(:,:,i)-centers(:,:,j);
        pair_clearance = min(pair_clearance,hypot(d(1,:),d(2,:))- ...
            scene.vehicles(i).radius-scene.vehicles(j).radius);
    end
end
clearance = min(clearance,pair_clearance);
metrics = struct('minimum_clearance',min(clearance),'clearance',clearance, ...
    'pair_clearance',pair_clearance,'max_defect',max_defect, ...
    'unweighted_kinematic',kinematic,'hard_violation',hard_violation);
if require_feasible
    assert(metrics.minimum_clearance>=-1e-6,'AIM:Collision','Discrete collision detected.');
    assert(hard_violation<=1e-6,'AIM:Boundary','State bounds or endpoint constraints violated.');
    assert(max_defect<=1e-6,'AIM:Dynamics','Excessive bicycle-model defect.');
end
end
