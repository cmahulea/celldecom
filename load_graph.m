function problem = load_graph(filename)

S = readstruct(filename);

problem.index = S.problem_index;
if isnumeric(S.index_refined)
    problem.refined = S.index_refined;
else
    problem.indices_refined=[];
end

problem.limits = eval(getfield(S,'limits'));

problem.obstacles={};
cont = 1;ind=1;
while cont
    if isfield(S,sprintf('O%d',ind))
        problem.obstacles{end+1} = eval(getfield(S,sprintf('O%d',ind)));
        ind = ind + 1;
    else
        cont = 0;
    end
end


problem.C={};
cont = 1;ind=1;
while cont
    if isfield(S,sprintf('C%d',ind))
        problem.C{end+1} = eval(getfield(S,sprintf('C%d',ind)));
        ind = ind + 1;
    else
        cont = 0;
    end
end

centroids = eval(getfield(S,'centroids'));
problem.centroid = {};
for i = 1 : size(centroids,1)
    problem.centroid{end+1} = centroids(i,:);
end


problem.adj = eval(getfield(S,'adj'));

problem.initial_points = eval(getfield(S,'initial_points'));
problem.final_points = eval(getfield(S,'final_points'));


return