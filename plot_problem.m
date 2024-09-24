function plot_problem(PN,prob)

figure;hold on;
C = PN.Q;
centr = PN.centroids;
for i=1:length(C)
    fill(C{i}(1,:),C{i}(2,:),'w','EdgeColor',[.8 .8 .8],'FaceAlpha',0.5);
    text(centr{i}(1),centr{i}(2),sprintf('p_{%d}',i),'HorizontalAlignment','center','Color','k','FontSize',10,'FontAngle','italic','FontName','TimesNewRoman');
end

initial_cells = find(PN.problems{prob}.m0);
for i=1:length(initial_cells)
   fill(C{initial_cells(i)}(1,:),C{initial_cells(i)}(2,:),'w','FaceColor','red','FaceAlpha',0.5);
end

final_cells = find(PN.problems{prob}.mf);
for i=1:length(final_cells)
   fill(C{final_cells(i)}(1,:),C{final_cells(i)}(2,:),'w','FaceColor','blue','FaceAlpha',0.5);
end


for i = 1 : length(PN.problems{prob}.traj)
    traj = PN.problems{prob}.traj{i};
    for j = 1 : length(traj)-1
        cent1 = PN.centroids{traj(j)};
        cent2 = PN.centroids{traj(j+1)};
        line([cent1(1) cent2(1)],[cent1(2) cent2(2)],'Color','green','LineWidth',2);
    end
end

objects = PN.obstacles;
for i = 1 : length(objects)
    %    plot(objects{i},'FaceColor','black');
    plot(polyshape(objects{i}.vertices'),'FaceColor','black','FaceAlpha',0.5);
end
xlim([0 PN.limits(1)]);
ylim([0 PN.limits(2)]);
title(sprintf('Problem %d. \nAverage cells: %f\nAverage dist. %f\nCongestion %f.',...
    prob,PN.problems{prob}.cell_length,...
    PN.problems{prob}.Euclidean),...
    PN.problems{prob}.congestion);