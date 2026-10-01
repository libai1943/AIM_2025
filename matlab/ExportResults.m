function ExportResults(scene,result,output)
%EXPORTRESULTS Save numerical outputs in the same schema as the C++ version.
if ~isfolder(output), mkdir(output); end
p = scene.parameters;
rows = zeros(numel(scene.vehicles)*p.nodes,11);
for i = 1:numel(scene.vehicles)
    indices = (i-1)*p.nodes+(1:p.nodes);
    rows(indices,:) = [repmat(i,p.nodes,1),(0:p.nodes-1)'*p.dt,result.fleet(:,:,i)'];
end
WriteCSV(fullfile(output,'trajectories.csv'), ...
    'vehicle,t,x,y,v,a,phi,omega,theta,xc,yc',rows);
WriteCSV(fullfile(output,'iterations.csv'), ...
    'iteration,penalty,Jnorm,Jkinematics,Jslack,infeasibility',result.iterations);
save(fullfile(output,'solution.mat'),'scene','result');
end

function WriteCSV(filename,header,values)
file = fopen(filename,'w');
assert(file>=0,'AIM:Output','Cannot create %s.',filename);
cleanup = onCleanup(@()fclose(file)); %#ok<NASGU>
fprintf(file,'%s\n',header);
format = [repmat('%.17g,',1,size(values,2)-1),'%.17g\n'];
fprintf(file,format,values');
end
