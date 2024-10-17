function plot_dataset(data)


figure;hold on;
for i = 1 : size(data.environment,1)
    for j = 1 : size(data.environment,2)
        if (data.environment(i,j) > 0)
            fill([i-1 i i i-1],[j-1 j-1 j j],'w','EdgeColor',[.8 .8 .8],'FaceAlpha',0.5);
            text(i-0.5,j-0.5,sprintf('%d',data.environment(i,j)),'HorizontalAlignment','center','Color','k','FontSize',10,'FontAngle','italic','FontName','TimesNewRoman');
        else
            fill([i-1 i i i-1],[j-1 j-1 j j],'k');
        end
    end
end

for i = 1 : size(data.start,1)
    if (data.start(i,1) > 0)
        fill([data.start(i,1)-1 data.start(i,1) data.start(i,1) data.start(i,1)-1],...
            [data.start(i,2)-1 data.start(i,2)-1 data.start(i,2) data.start(i,2)],'r','EdgeColor',[.8 .8 .8],'FaceAlpha',0.5);
    end
end

for i = 1 : size(data.end,1)
    if (data.end(i,1) > 0)
        fill([data.end(i,1)-1 data.end(i,1) data.end(i,1) data.end(i,1)-1],...
            [data.end(i,2)-1 data.end(i,2)-1 data.end(i,2) data.end(i,2)],'b','EdgeColor',[.8 .8 .8],'FaceAlpha',0.5);
    end
end

for i = 1 : length(data.traj)
    traj = data.traj{i};
    for j = 1 : size(traj,1)-1
        line([traj(j,1) traj(j+1,1)],[traj(j,2) traj(j+1,2)],'Color','green','LineWidth',2);
    end
end

if isempty(data.refined)
    title_fig = sprintf('Problem %d (not refined). \nAverage dist. %f\nCongestion %f.\nCell number:%d.',...
        data.index,data.distance,data.congestion,data.num_cells);
else
    title_fig = sprintf('Problem %d (refined %d). \nAverage dist. %f\nCongestion %f.\nCell number:%d.',...
        data.index,data.refined,data.distance,data.congestion,data.num_cells);
end
title(title_fig);
xlim([0 32]);
ylim([0 32]);
return
