function figures = PlotResults(scene,result,output)
%PLOTRESULTS Two static figures: cooperative motion, then state/control profiles.
% Every vehicle body in a snapshot is evaluated at the same time index.
p = scene.parameters;
t = linspace(0,p.horizon,p.nodes);
colors = [0.10,0.36,0.72;0.00,0.58,0.66;0.10,0.60,0.40;0.53,0.66,0.13; ...
    0.91,0.65,0.08;0.94,0.38,0.12;0.78,0.17,0.26;0.64,0.21,0.56; ...
    0.42,0.29,0.69;0.22,0.35,0.45];
figures(1) = figure('Color','w','Name','Ten-vehicle cooperation', ...
    'Position',[60,60,1320,780]);
layout = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
ax = nexttile(layout,1,[2,2]);
extent = 10*ceil(max(abs(reshape(result.fleet(1:2,:,:),[],1)))/10)+10;
DrawMap(ax,scene,[-extent,extent,-extent,extent]);
% Footprints go down first; trajectory lines remain visible above them.
for i = 1:10
    for k = 1:20:p.nodes
        DrawBody(ax,scene.vehicles(i),result.fleet(:,k,i),colors(i,:),0.08);
    end
end
paths = gobjects(10,1);
for i = 1:10
    q = result.fleet(:,:,i);
    paths(i) = plot(ax,q(1,:),q(2,:),'Color',colors(i,:),'LineWidth',1.6);
    plot(ax,q(1,1),q(2,1),'o','Color',colors(i,:),'MarkerFaceColor','w','LineWidth',1.5);
    text(ax,q(1,1)+3,q(2,1)+4,sprintf('%02d',i),'Color',colors(i,:), ...
        'FontWeight','bold','FontSize',10);
end
title(ax,'Ten CAVs, one joint trajectory optimization','FontWeight','bold');
legend(ax,paths,compose('CAV %02d',1:10),'NumColumns',5,'Location','southoutside','Box','off');
for panel = 1:2
    ax = nexttile(layout,3*panel);
    DrawMap(ax,scene,[-48,48,-48,48]);
    k = 1+round((9+6*(panel-1))/p.dt);
    for i = 1:10
        q = result.fleet(:,:,i);
        plot(ax,q(1,:),q(2,:),':','Color',0.7+0.3*colors(i,:),'LineWidth',1);
        DrawBody(ax,scene.vehicles(i),q(:,k),colors(i,:),0.72);
        if max(abs(q(1:2,k)))<43
            text(ax,q(8,k),q(9,k),sprintf('%d',i),'HorizontalAlignment','center', ...
                'Color','w','FontWeight','bold','FontSize',9);
        end
    end
    title(ax,sprintf('Synchronized snapshot | t = %.1f s',t(k)));
end
title(layout,'LANE-FREE COOPERATIVE PLANNING  |  Chapter 10', ...
    'FontWeight','bold','FontSize',17,'Color',[0.12,0.19,0.28]);

figures(2) = figure('Color','w','Name','Optimal states and controls', ...
    'Position',[90,80,1240,750]);
layout = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
components = [3,4,5,6,7];
labels = {'Speed (m/s)','Acceleration (m/s^2)','Steering angle (rad)', ...
    'Steering rate (rad/s)','Heading (rad)'};
for panel = 1:5
    ax = nexttile(layout); hold(ax,'on');
    for i = 1:10
        plot(ax,t,result.fleet(components(panel),:,i),'Color',colors(i,:),'LineWidth',1.2);
    end
    xlabel(ax,'Time (s)'); ylabel(ax,labels{panel});
    grid(ax,'on'); box(ax,'off'); ax.FontSize = 10;
end
ax = nexttile(layout); hold(ax,'on');
plot(ax,t,result.metrics.pair_clearance,'Color',[0.08,0.48,0.62],'LineWidth',1.6);
plot(ax,t,result.metrics.clearance,'Color',[0.88,0.35,0.17],'LineWidth',1.4);
yline(ax,0,'k:'); xlabel(ax,'Time (s)'); ylabel(ax,'Covering-disc clearance (m)');
legend(ax,{'Between CAVs','Including boundaries'},'Location','best','Box','off');
grid(ax,'on'); box(ax,'off');
title(layout,'STATE, CONTROL AND CLEARANCE PROFILES  |  10 CAVs', ...
    'FontWeight','bold','FontSize',17,'Color',[0.12,0.19,0.28]);
if nargin>=3 && ~isempty(output)
    if ~isfolder(output), mkdir(output); end
    exportgraphics(figures(1),fullfile(output,'cooperative_trajectories.png'),'Resolution',180);
    exportgraphics(figures(2),fullfile(output,'state_control_profiles.png'),'Resolution',180);
end
end

function DrawMap(ax,scene,limits)
hold(ax,'on'); axis(ax,'equal'); axis(ax,limits);
ax.Color = [0.96,0.97,0.98];
for b = scene.obstacles'
    patch(ax,b([1,2,2,1]),b([3,3,4,4]),[0.83,0.86,0.88], ...
        'EdgeColor',[0.62,0.68,0.72],'LineWidth',0.8);
end
xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
box(ax,'off'); ax.FontSize = 10; ax.Layer = 'top';
end

function DrawBody(ax,vehicle,q,color,opacity)
local = [-vehicle.LR,vehicle.LW+vehicle.LF,vehicle.LW+vehicle.LF,-vehicle.LR; ...
    -vehicle.LB/2,-vehicle.LB/2,vehicle.LB/2,vehicle.LB/2];
rotation = [cos(q(7)),-sin(q(7));sin(q(7)),cos(q(7))];
body = rotation*local+q(1:2);
patch(ax,body(1,:),body(2,:),color,'FaceAlpha',opacity,'EdgeColor',color, ...
    'EdgeAlpha',max(opacity,0.2),'LineWidth',0.7,'HandleVisibility','off');
end
