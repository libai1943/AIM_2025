function result = OptimizeLIOM(scene,reference)
%OPTIMIZELIOM Joint CasADi/IPOPT/MA27 optimization, Eqs. (26)-(29).
% All hard constraints are linear. Nonlinear differential and center
% equations enter the objective as squared residuals.
import casadi.*
p = scene.parameters;
nv = numel(scene.vehicles);
state_count = 9*p.nodes*nv;
weight = p.penalty;
history = zeros(p.max_iterations,6);
for iteration = 1:p.max_iterations
    fprintf('LIOM %d: constructing STC/RSTC corridors...\n',iteration);
    corridors = BuildCorridors(scene,reference);
    np = size(corridors.pairs,1);
    z = SX.sym('expanded_state_and_slack',state_count+np*p.nodes,1);
    lower = -1e20*ones(size(z));
    upper = -lower;
    guess = [reference(:);zeros(np*p.nodes,1)];
    lb = -1e20*ones(9,p.nodes,nv);
    ub = -lb;
    normal = SX(0);
    residuals = cell(nv,1);
    constraints = {};
    gl = []; gu = [];
    for i = 1:nv
        vehicle = scene.vehicles(i);
        indices = (i-1)*9*p.nodes+(1:9*p.nodes);
        q = reshape(z(indices),9,p.nodes);
        lb(3,:,i) = p.vmin; ub(3,:,i) = p.vmax;
        lb(4:6,:,i) = repmat(-[p.amax;p.phimax;p.omegamax],1,p.nodes);
        ub(4:6,:,i) = -lb(4:6,:,i);
        box = corridors.absolute(:,:,i);
        lb(8:9,:,i) = box([1,3],:); ub(8:9,:,i) = box([2,4],:);
        geometry = [q(8,:)-q(1,:)-vehicle.offset*cos(q(7,:)); ...
            q(9,:)-q(2,:)-vehicle.offset*sin(q(7,:))];
        q0 = q(:,1:end-1);
        dynamic = (q([1,2,3,5,7],2:end)-q0([1,2,3,5,7],:))/p.dt - ...
            [q0(3,:).*cos(q0(7,:));q0(3,:).*sin(q0(7,:)); ...
            q0(4,:);q0(6,:);q0(3,:).*tan(q0(5,:))/vehicle.LW];
        residuals{i} = [geometry(:);dynamic(:)];
        normal = normal+p.smoothness*p.dt*sumsqr(q0([4,6],:)) + ...
            sumsqr(q(1:2,end)-vehicle.desired);
        lb(1:7,1,i) = vehicle.initial(1:7);
        ub(1:7,1,i) = vehicle.initial(1:7);
        lb([3:7],end,i) = [vehicle.exit_speed;0;0;0;vehicle.heading];
        ub([3:7],end,i) = lb([3:7],end,i);
        delta = q(1:2,end)-vehicle.exit;
        u = [cos(vehicle.heading);sin(vehicle.heading)];
        u(abs(u)<1e-12) = 0;
        constraints{end+1} = [u'*delta;[-u(2),u(1)]*delta]; %#ok<AGROW>
        gl = [gl;0;0]; gu = [gu;1e20;0]; %#ok<AGROW>
    end
    lower(1:state_count) = lb(:); upper(1:state_count) = ub(:);
    fixed = lower==upper;
    guess(fixed) = lower(fixed);
    for pair = 1:np
        vehicles = corridors.pairs(pair,:);
        a = reshape(z((vehicles(1)-1)*9*p.nodes+(1:9*p.nodes)),9,p.nodes);
        b = reshape(z((vehicles(2)-1)*9*p.nodes+(1:9*p.nodes)),9,p.nodes);
        indices = state_count+(pair-1)*p.nodes+(1:p.nodes);
        slack = z(indices)';
        lower(indices) = 0;
        relative = a(8:9,:)-b(8:9,:);
        box = corridors.relative(:,:,pair);
        expression = [relative(1,:)+slack;relative(1,:)-slack; ...
            relative(2,:)+slack;relative(2,:)-slack];
        low = [box(1,:);-1e20*ones(1,p.nodes);box(3,:);-1e20*ones(1,p.nodes)];
        high = [1e20*ones(1,p.nodes);box(2,:);1e20*ones(1,p.nodes);box(4,:)];
        constraints{end+1} = expression(:); %#ok<AGROW>
        gl = [gl;low(:)]; gu = [gu;high(:)]; %#ok<AGROW>
        xy = reference(8:9,:,vehicles(1))-reference(8:9,:,vehicles(2));
        guess(indices) = MinimumSlack(xy,box);
    end
    residual = vertcat(residuals{:});
    kinematic = p.dt*sumsqr(residual);
    slack_cost = p.slack_weight*sum1(z(state_count+1:end));
    objective = kinematic+(normal+slack_cost)/weight;
    g = vertcat(constraints{:});
    jac = jacobian(residual,z);
    curvature = 2*p.dt*(jac'*jac)+hessian(normal,z)/weight;
    sigma = SX.sym('objective_multiplier');
    multipliers = SX.sym('constraint_multipliers',numel(g));
    options = struct('print_time',false);
    options.ipopt = struct('print_level',0,'sb','yes','tol',1e-10, ...
        'acceptable_iter',0,'max_iter',3000,'max_cpu_time',180, ...
        'linear_solver','ma27','bound_relax_factor',0,'nlp_scaling_method','none');
    options.ipopt.mu_strategy = 'adaptive';
    options.ipopt.ma27_pivtol = 1e-6;
    hsl = getenv('HSL_LIBRARY');
    if weight>=1e8, options.ipopt.tol = 1e-12; end
    if ~isempty(hsl), options.ipopt.hsllib = hsl; end
    options.hess_lag = Function('gauss_newton', ...
        {z,SX.sym('unused_parameters',0),sigma,multipliers},{triu(sigma*curvature)});
    fprintf('LIOM %d: solving %d variables, %d constraints with MA27 (%d interacting pairs)...\n', ...
        iteration,numel(z),numel(g),np);
    solver = nlpsol('liom','ipopt',struct('x',z,'f',objective,'g',g),options);
    answer = solver('x0',guess,'lbx',lower,'ubx',upper,'lbg',gl,'ubg',gu);
    status = solver.stats();
    assert(status.success,'AIM:Solver','LIOM NLP failed: %s',status.return_status);
    values = full(answer.x);
    reference = reshape(values(1:state_count),9,p.nodes,nv);
    for pair = 1:np
        vehicles = corridors.pairs(pair,:);
        xy = reference(8:9,:,vehicles(1))-reference(8:9,:,vehicles(2));
        indices = state_count+(pair-1)*p.nodes+(1:p.nodes);
        values(indices) = MinimumSlack(xy,corridors.relative(:,:,pair));
    end
    terms = Function('terms',{z},{normal,weight*kinematic,slack_cost,g});
    [jn,jk,js,hard] = terms(values);
    hard = full(hard);
    violation = max([0;lower-values;values-upper;gl-hard;hard-gu]);
    assert(violation<=1e-6,'AIM:LinearConstraints','Hard linear constraint violation.');
    jn = full(jn); jk = full(jk); js = max(0,full(js));
    metrics = ValidateSolution(scene,reference,false);
    history(iteration,:) = [iteration,weight,jn,jk,js,jk+js];
    fprintf('LIOM %d: normal=%.8g, Jkin=%.8g, Jslack=%.8g, clearance=%.6g m\n', ...
        iteration,jn,jk,js,metrics.minimum_clearance);
    if jk+js<=p.tolerance && metrics.minimum_clearance>=-1e-6
        result = struct('fleet',reference,'iterations',history(1:iteration,:), ...
            'metrics',ValidateSolution(scene,reference,true));
        return;
    end
    weight = weight*200;
end
error('AIM:Convergence','LIOM did not reach the weighted infeasibility threshold.');
end

function slack = MinimumSlack(xy,box)
slack = max([zeros(1,size(xy,2));box(1,:)-xy(1,:);xy(1,:)-box(2,:); ...
    box(3,:)-xy(2,:);xy(2,:)-box(4,:)],[],1);
end
