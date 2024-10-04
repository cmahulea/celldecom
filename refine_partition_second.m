function newPart = refine_partition_second(PN)

    newPart.Q = PN.Q;
    newPart.adj = PN.adj;
    newPart.centroids = PN.centroids;
    newPart.obstacles = PN.obstacles;
    newPart.limits = PN.limits;
    regions_in_trajectories = [];
    trajs = PN.traj;
    for j = 1 : length(trajs)
        regions_in_trajectories = [regions_in_trajectories trajs{j}];
    end
    regions_in_trajectories = unique(regions_in_trajectories,'sorted');
    for j = length(regions_in_trajectories): -1 : 1 %fora each region on a trajectory partition it again
        adjacent_regions = setdiff(find(newPart.adj(regions_in_trajectories(j),:)),regions_in_trajectories(j));
        %remove the adjancy of the region that is refined
        vertices = PN.Q{regions_in_trajectories(j)};
        x1 = vertices(1,1);
        x2 = vertices(1,2);
        y1 = vertices(2,1);
        y2 = vertices(2,3);%[x1 x2 x2 x1;y1 y1 y2 y2]
        if ((abs(x2-x1) <=1) || (abs(y2-y1) <=1)) %if precision has been reached go to the next region
            regions_in_trajectories(j) = [];
            continue;
        end
        newPart.Q{end+1}=[x1, (x1+x2)/2, (x1+x2)/2, x1;y1, y1, (y1+y2)/2, (y1+y2)/2]; %lower-left rectangle
        %fill(newPart{i}.Q{end}(1,:),newPart{i}.Q{end}(2,:),'w','FaceColor','magenta','FaceAlpha',0.5);
        newPart.centroids{end+1} = mean(newPart.Q{end},2)';

        newPart.Q{end+1}= [(x1+x2)/2, x2, x2, (x1+x2)/2; y1, y1, (y1+y2)/2,(y1+y2)/2]; %lower-right rectangle
        %fill(newPart{i}.Q{end}(1,:),newPart{i}.Q{end}(2,:),'w','FaceColor','magenta','FaceAlpha',0.5);
        newPart.centroids{end+1} = mean(newPart.Q{end},2)';

        newPart.Q{end+1} =[(x1+x2)/2, x2, x2, (x1+x2)/2; (y1+y2)/2, (y1+y2)/2, y2, y2]; %upper-right rectangle
        %fill(newPart{i}.Q{end}(1,:),newPart{i}.Q{end}(2,:),'w','FaceColor','magenta','FaceAlpha',0.5);
        newPart.centroids{end+1} = mean(newPart.Q{end},2)';

        newPart.Q{end+1} =[x1, (x1+x2)/2, (x1+x2)/2, x1; (y1+y2)/2, (y1+y2)/2, y2, y2]; %upper-left rectangle
        %fill(newPart{i}.Q{end}(1,:),newPart{i}.Q{end}(2,:),'w','FaceColor','magenta','FaceAlpha',0.5);
        newPart.centroids{end+1} = mean(newPart.Q{end},2)';
        length_Q = length(newPart.Q);
        newPart.adj(length_Q-3,length_Q-2) = 1;
        newPart.adj(length_Q-3,length_Q) = 1;
        newPart.adj(length_Q-2,length_Q-3) = 1;
        newPart.adj(length_Q-2,length_Q-1) = 1;
        newPart.adj(length_Q-1,length_Q-2) = 1;
        newPart.adj(length_Q-1,length_Q) = 1;
        newPart.adj(length_Q,length_Q-1) = 1;
        newPart.adj(length_Q,length_Q-3) = 1;
        newPart.adj(length_Q,length_Q) = 1;
        newPart.adj(length_Q-1,length_Q-1) = 1;
        newPart.adj(length_Q-2,length_Q-2) = 1;
        newPart.adj(length_Q-3,length_Q-3) = 1;

        Ak=[0 -1;-1 0;0 1;1 0]; %matrix A for H-repres of each rectangle
        for k=1:length(adjacent_regions)
            for ll=0:3
                bi=[newPart.Q{adjacent_regions(k)}(2,1), newPart.Q{adjacent_regions(k)}(1,1), ...
                    -newPart.Q{adjacent_regions(k)}(2,3),...
                    -newPart.Q{adjacent_regions(k)}(1,2)]';    %H-repres of rectangle i (y1,x1,-y2,-x2) - based on teh way vertices were added in cell C
                bj=[newPart.Q{end-ll}(2,1), newPart.Q{end-ll}(1,1),...
                    -newPart.Q{end-ll}(2,3), -newPart.Q{end-ll}(1,2)]';    %H-repres of rectangle j
                V.V = H2V_conversion([Ak;Ak],[-bi;-bj])';
                if size(V.V,1) >=1 %==2   %rectangles i and j are adjacent iff their intersection is a line segment (2 vertces)
                    newPart.adj(adjacent_regions(k),length(newPart.Q)-ll)=1;
                    newPart.adj(length(newPart.Q)-ll,adjacent_regions(k))=1;
                end
            end
        end

    end
    for j = length(regions_in_trajectories): -1 : 1 %fora each region on a trajectory partition it again
        newPart.adj(regions_in_trajectories(j),:) = [];
        newPart.adj(:,regions_in_trajectories(j)) = [];
        newPart.Q(regions_in_trajectories(j)) = [];
        newPart.centroids(regions_in_trajectories(j)) = [];
    end
    newPart.no_refined_cells = length(regions_in_trajectories);
%    fprintf(1,'\nRefine cells %s',mat2str(regions_in_trajectories));
return