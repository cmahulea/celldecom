function plot_problem_file(filename)

problem = load_problem(filename);

figure;hold on;
C = problem.Q;

all_fused = [];
for i = 1 : length(problem.fuse)
    fused = problem.fuse{i};
    all_fused = [all_fused fused];
    pol = polyshape();
    for j = 1 : length(fused)
        pol = union(pol,polyshape(C{fused(j)}'));
    end
    plot(pol,'Facecolor','yellow');
    plot(problem.fuse_centroid{i}(1),problem.fuse_centroid{i}(2),'*r');
end

centr = problem.centroids;
for i=1:length(C)
    if isempty(find(all_fused == i))
        fill(C{i}(1,:),C{i}(2,:),'w','EdgeColor',[.8 .8 .8],'FaceAlpha',0.5);
        text(centr{i}(1),centr{i}(2),sprintf('p_{%d}',i),'HorizontalAlignment','center','Color','k','FontSize',10,'FontAngle','italic','FontName','TimesNewRoman');
    end
end


initial_cells = find(problem.m0);
for i=1:length(initial_cells)
   fill(C{initial_cells(i)}(1,:),C{initial_cells(i)}(2,:),'w','FaceColor','red','FaceAlpha',0.5);
end

final_cells = find(problem.mf);
for i=1:length(final_cells)
   fill(C{final_cells(i)}(1,:),C{final_cells(i)}(2,:),'w','FaceColor','blue','FaceAlpha',0.5);
end


for i = 1 : length(problem.traj)
    traj = problem.traj{i};
    for j = 1 : length(traj)-1
        cent1 = problem.centroids{traj(j)};
        cent2 = problem.centroids{traj(j+1)};
        line([cent1(1) cent2(1)],[cent1(2) cent2(2)],'Color','green','LineWidth',2);
    end
end

objects = problem.obstacles;
for i = 1 : length(objects)
    %    plot(objects{i},'FaceColor','black');
    plot(polyshape(objects{i}'),'FaceColor','black','FaceAlpha',0.5);
end
xlim([0 problem.limits(1)]);
ylim([0 problem.limits(2)]);
if isempty(problem.indices_refined)
    title_fig = sprintf('Problem %d (not refined). \nAverage cells: %f\nAverage dist. %f\nCongestion %f.',...
        problem.index,problem.cell_length,...
        problem.Euclidean,problem.congestion);
else
    title_fig = sprintf('Problem %d (refined %d). \nAverage cells: %f\nAverage dist. %f\nCongestion %f.',...
        problem.index,problem.indices_refined,problem.cell_length,...
        problem.Euclidean,problem.congestion);
end
title(title_fig);