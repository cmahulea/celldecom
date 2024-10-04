function problem = load_problem(filename)

S = readstruct(filename);
problem.index = S.problem_index;
if isnumeric(S.index_refined)
    problem.indices_refined = S.index_refined;
else
    problem.indices_refined=[];
end
problem.Pre = eval(S.Pre);
problem.Post = eval(S.Post);
problem.C = eval(S.C);
problem.m0 = eval(S.m0);
problem.mf = eval(S.mf);
problem.sigma = S.sigma;
problem.traj={};
cont = 1;ind=1;
while cont
    if isfield(S,sprintf('T%d',ind))
        problem.traj{end+1} = eval(getfield(S,sprintf('T%d',ind)));
        ind = ind + 1;
    else
        cont = 0;
    end
end
problem.Q={};
cont = 1;ind=1;
while cont
    if isfield(S,sprintf('Q%d',ind))
        problem.Q{end+1} = eval(getfield(S,sprintf('Q%d',ind)));
        ind = ind + 1;
    else
        cont = 0;
    end
end
problem.fuse={};
cont = 1;ind=1;
while cont
    if isfield(S,sprintf('F%d',ind))
        problem.fuse{end+1} = eval(getfield(S,sprintf('F%d',ind)));
        ind = ind + 1;
    else
        cont = 0;
    end
end

problem.fuse_centroid={};
cont = 1;ind=1;

while cont
    if isfield(S,sprintf('FC%d',ind))
        problem.fuse_centroid{end+1} = eval(getfield(S,sprintf('FC%d',ind)));
        ind = ind + 1;
    else
        cont = 0;
    end
end

problem.adj = eval(getfield(S,'adj'));

centroids = eval(getfield(S,'centroids'));
problem.centroids = {};
for i = 1 : size(centroids,1)
    problem.centroids{end+1} = centroids(i,:);
end

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


problem.limits = eval(getfield(S,'limits'));
problem.congestion = getfield(S,'congestion');
problem.time_solving_LP = getfield(S,'time_solving_LP');
if isfield(S,'problem.time_solving_LP2')
    problem.problem.time_solving_LP2 = getfield(S,'problem.time_solving_LP2');
end
problem.cell_length = getfield(S,'cell_length');
problem.Euclidean = getfield(S,'Euclidean');

return