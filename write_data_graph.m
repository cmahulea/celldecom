function flag = write_data_graph(filename,graph)

flag = 1;


struct_to_write.problem_index = graph.index;
struct_to_write.index_refined = num2str(graph.refined);

for i = 1 : length(graph.C)
    struct_to_write = setfield(struct_to_write,sprintf('C%d',i),mat2str(graph.C{i}));
end

struct_to_write.adj = mat2str(graph.adj);

Q = [];
for i = 1 : length(graph.centroid)
    Q(i,:) = graph.centroid{i};
end
struct_to_write.centroids = mat2str(Q);

for i = 1 : length(graph.obstacles)
    struct_to_write = setfield(struct_to_write,sprintf('O%d',i),mat2str(graph.obstacles{i}));
end
struct_to_write.limits = mat2str(graph.limits);

struct_to_write.initial_points = mat2str(graph.initial_points);
struct_to_write.final_points = mat2str(graph.final_points);


writestruct(struct_to_write,filename);
