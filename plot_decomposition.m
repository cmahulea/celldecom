function plot_decomposition(PN)


xmax = PN.limits(1);
ymax = PN.limits(2);
figure;axis([0 xmax 0 ymax]);
hold on;
grid on;

for i = 1 : length(PN.obstacles)
    plot(polyshape(PN.obstacles{i}'),'FaceColor','black');
end
%represent cells:
for i=1:length(PN.Q)
    centr=PN.centroids{i};
    fill(PN.Q{i}(1,:),PN.Q{i}(2,:),'w','EdgeColor',[.8 .8 .8],'FaceAlpha',0.5);
    text(centr(1),centr(2),sprintf('p_{%d}',i),'HorizontalAlignment','center','Color','k','FontSize',10,'FontAngle','italic','FontName','TimesNewRoman');
end

initial_cells = PN.potential_m0;
for i=1:length(initial_cells)
    fill(PN.Q{initial_cells(i)}(1,:),PN.Q{initial_cells(i)}(2,:),'w','FaceColor','red','FaceAlpha',0.5);
end
final_cells = PN.potential_mf;
for i=1:length(final_cells)
    fill(PN.Q{final_cells(i)}(1,:),PN.Q{final_cells(i)}(2,:),'w','FaceColor','green','FaceAlpha',0.5);
end
