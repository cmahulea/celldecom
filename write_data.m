function flag = write_data(filename,problem)

flag = 1;

struct_to_write.problem_index = problem.index;
struct_to_write.index_refined = num2str(problem.indices_refined);
struct_to_write.Pre = mat2str(problem.Pre);
struct_to_write.Post = mat2str(problem.Post);
struct_to_write.C = mat2str(problem.C);
struct_to_write.m0 = mat2str(problem.m0);
struct_to_write.mf = mat2str(problem.mf);
struct_to_write.sigma = mat2str(problem.sigma);
struct_to_write.initial_points = mat2str(problem.initial_points);
struct_to_write.final_points = mat2str(problem.final_points);
for i = 1 : length(problem.traj)
    struct_to_write = setfield(struct_to_write,sprintf('T%d',i),mat2str(problem.traj{i}));
end
for i = 1 : length(problem.Q)
    struct_to_write = setfield(struct_to_write,sprintf('Q%d',i),mat2str(problem.Q{i}));
end
if isfield(problem,'fuse')
    for i = 1 : length(problem.fuse)
        struct_to_write = setfield(struct_to_write,sprintf('F%d',i),mat2str(problem.fuse{i}));
        struct_to_write = setfield(struct_to_write,sprintf('FC%d',i),mat2str(problem.fuse_centroid{i}));
    end
end
struct_to_write.adj = mat2str(problem.adj);
Q = [];
for i = 1 : length(problem.centroids)
    Q(i,:) = problem.centroids{i};
end
struct_to_write.centroids = mat2str(Q);
for i = 1 : length(problem.obstacles)
    struct_to_write = setfield(struct_to_write,sprintf('O%d',i),mat2str(problem.obstacles{i}));
end
struct_to_write.limits = mat2str(problem.limits);
struct_to_write.congestion = num2str(problem.congestion);
struct_to_write.time_solving_LP = num2str(problem.time_solving_LP);
if isfield(problem,'time_solving_LP2')
    struct_to_write.time_solving_LP2 = num2str(problem.time_solving_LP2);
end
struct_to_write.cell_length = num2str(problem.cell_length);
struct_to_write.Euclidean = num2str(problem.Euclidean);

writestruct(struct_to_write,filename);

