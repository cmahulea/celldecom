function PN = partition_env(xmax,ymax,objects,decomposition,messages)


if messages
    fprintf(1,'\n\n Start partioning the environment with %s cell decomposition.',decomposition);
end
tic;
switch decomposition
    case 'rectangular'
        [C,adj,new_obstacles] = rectangular_decomposition(objects,[0, xmax, 0, ymax]);
    case 'triangular'
        [C,adj,new_obstacles] = triangular_decomposition(objects,[0, xmax, 0, ymax]);
end
if messages
    fprintf(1,'\n Cell decomposition completed. Time to compute it: %f',toc)
end

centroids = cell(1,length(C));
for i=1:length(C)
    centroids{i} = mean(C{i},2)';
end

fprintf(1,'\n\n Start constructing the Petri net.');
tic;
PN = construct_PN(C,adj,ymax);
fprintf(1,'\n Petri net constructed. Time to compute it: %f\n',toc)

PN.Q = C;
PN.adj = adj;
PN.centroids = centroids;
PN.obstacles = new_obstacles;
PN.limits = [xmax,ymax];

