function corridors = BuildCorridors(scene,reference)
%BUILDCORRIDORS Algorithms 1-2: absolute STCs and pairwise relative STCs.
p = scene.parameters;
nv = numel(scene.vehicles);
corridors.absolute = zeros(4,p.nodes,nv);
corridors.pairs = zeros(0,2);
corridors.relative = zeros(4,p.nodes,0);
for i = 1:nv
    radius = scene.vehicles(i).radius;
    safe = @(box) AbsoluteSafe(box,radius,scene);
    for k = 1:p.nodes
        corridors.absolute(:,k,i) = ExpandBox(reference(8:9,k,i),p,safe);
    end
end
for i = 1:nv
    for j = i+1:nv
        radius = scene.vehicles(i).radius+scene.vehicles(j).radius;
        if all(BoxDistance(corridors.absolute(:,:,i),corridors.absolute(:,:,j))>radius)
            continue;
        end
        pair = size(corridors.pairs,1)+1;
        corridors.pairs(pair,:) = [i,j];
        safe = @(box) hypot(max([box(1),-box(2),0]), ...
            max([box(3),-box(4),0])) >= radius+1e-8;
        for k = 1:p.nodes
            seed = reference(8:9,k,i)-reference(8:9,k,j);
            corridors.relative(:,k,pair) = ExpandBox(seed,p,safe);
        end
    end
end
end

function safe = AbsoluteSafe(box,radius,scene)
w = scene.workspace;
safe = box(1)>=w(1)+radius && box(2)<=w(2)-radius && ...
    box(3)>=w(3)+radius && box(4)<=w(4)-radius;
if safe
    safe = all(BoxDistance(box,scene.obstacles')>=radius+1e-8);
end
end

function distance = BoxDistance(a,b)
dx = max(max(a(1,:)-b(2,:),b(1,:)-a(2,:)),0);
dy = max(max(a(3,:)-b(4,:),b(3,:)-a(4,:)),0);
distance = hypot(dx,dy);
end

function box = ExpandBox(seed,p,safe)
box = seed([1,1,2,2]);
if ~safe(box)
    found = false;
    for ring = 1:200
        for a = 0:31
            trial = seed+ring*0.05*[cos(2*pi*a/32);sin(2*pi*a/32)];
            candidate = trial([1,1,2,2]);
            if safe(candidate)
                box = candidate;
                found = true;
                break;
            end
        end
        if found, break; end
    end
    assert(found,'AIM:CorridorSeed','No free corridor seed in the local neighborhood.');
end
growth = zeros(1,4);
done = false(1,4);
signs = [-1,1,-1,1];
while ~all(done)
    for side = [4,3,1,2]
        if done(side), continue; end
        candidate = box;
        candidate(side) = candidate(side)+signs(side)*p.box_step;
        if growth(side)+p.box_step>p.box_limit || ~safe(candidate)
            done(side) = true;
        else
            box = candidate;
            growth(side) = growth(side)+p.box_step;
        end
    end
end
end
