function PNnew = refine_problems_rectangular(PN,filename)

%start refining until no more possible
still_refining = 1;
PNnew={};
refined_problems = length(PN.problems);

while (still_refining == 1)
    still_refining = 0;
    while ~isempty(PN.problems)
        if (PN.problems{1}.should_refine == 1)
            still_refining = 1;
            fprintf(1,'\n==================================================');
            refined_problems = refined_problems + 1;
            fprintf(1,'\n Problem %d.',refined_problems);
            newPart = refine_partition_second(PN.problems{1});
            PN.problems{1}.should_refine = 0; %do not refine again
            if (newPart.no_refined_cells == 0)
                fprintf(1,'\nThe refined problem is not new! Skip it\n==================================================');
                refined_problems = refined_problems - 1;
                continue;
            end
            PN.problems{end+1} = construct_PN(newPart.Q,newPart.adj,newPart.limits(2));
            PN.problems{1}.indices_refined = refined_problems;
            PN.problems{end}.Q = newPart.Q;
            PN.problems{end}.adj = newPart.adj;
            PN.problems{end}.obstacles = newPart.obstacles;
            PN.problems{end}.centroids = newPart.centroids;
            PN.problems{end}.limits = newPart.limits;
            PN.problems{end}.no_refined_cells = newPart.no_refined_cells;
            PN.problems{end}.indices_refined = [];
            PN.problems{end}.should_refine = 1;
            PN.problems{end}.index = refined_problems;
            for i = 1 : size(PN.problems{1}.initial_points,1)
                initial{i} = PN.problems{1}.initial_points(i,:);
                final{i} = PN.problems{1}.final_points(i,:);
            end
            m0 = sparse(length(PN.problems{end}.Q),1);
            mf = sparse(length(PN.problems{end}.Q),1);
            for j = 1 : length(initial)
                for k = 1 : length(PN.problems{end}.Q)
                    [in,on] = inpolygon(initial{j}(1),initial{j}(2),PN.problems{end}.Q{k}(1,:),PN.problems{end}.Q{k}(2,:));
                    if (in+on>=1)
                        m0(k) = m0(k) + 1;
                        break;
                    end
                end
            end
            for j = 1 : length(final)
                for k = 1 : length(PN.problems{end}.Q)
                    [in,on] = inpolygon(final{j}(1),final{j}(2),PN.problems{end}.Q{k}(1,:),PN.problems{end}.Q{k}(2,:));
                    if (in+on>=1)
                        mf(k) = mf(k) + 1;
                        break;
                    end
                end
            end
            %%%%%%% release memory
            PN.problems{end}.initial_points = PN.problems{1}.initial_points;
            PN.problems{end}.final_points = PN.problems{1}.final_points;

            %fuse the empty regions
            regions_to_not_consider = [];
            for i = 1 : length(PN.problems{1}.traj)
                regions_to_not_consider = [regions_to_not_consider PN.problems{1}.traj{i}];
            end
            regions_to_not_consider = unique(regions_to_not_consider,'sorted');
            more_regions_to_not_consider = [];
            adj_two_steps = PN.problems{1}.adj;% * PN.problems{1}.adj;
            for i = 1 : length(regions_to_not_consider)
                more_regions_to_not_consider = [more_regions_to_not_consider find(adj_two_steps(regions_to_not_consider(i),:))];
            end

            potential_merging_regions = sort(setdiff(1:length(PN.problems{1}.Q),more_regions_to_not_consider));
            PN.problems{1}.fuse = {};
            PN.problems{1}.fuse_centroid = {};
            while ~isempty(potential_merging_regions)
                index = randi([1,length(potential_merging_regions)],1); %index of the region that is tried to merge
                region = potential_merging_regions(index);
                PN.problems{1}.fuse{end+1} = region;
                PN.problems{1}.fuse_centroid{end+1} = [];
                add_more = 1;
                while add_more
                    add_more = 0;
                    pol2 = polyshape();
                    for i = 1 : length(PN.problems{1}.fuse{end})
                        pol2 = union(pol2, polyshape(PN.problems{1}.Q{PN.problems{1}.fuse{end}(i)}'));
                    end
                    adjacent_reg_possible = [];
                    for i = 1 : length(PN.problems{1}.fuse{end})
                        adjacent_reg_possible = [adjacent_reg_possible find(PN.problems{1}.adj(PN.problems{1}.fuse{end}(i),:))];
                    end
                    adjacent_reg_possible = setdiff(adjacent_reg_possible,PN.problems{1}.fuse{end});
                    adjacent_reg_possible = intersect(adjacent_reg_possible, potential_merging_regions);
                    for j = 1 : length(adjacent_reg_possible)
                        pol1 = polyshape( PN.problems{1}.Q{adjacent_reg_possible(j)}');
                        pol = union(pol1 , pol2);
                        if isempty(subtract(convhull(pol),pol).Vertices) %if the diference of the convex hull with the union of regions is empty then the union is convex
                            PN.problems{1}.fuse{end} = [PN.problems{1}.fuse{end} adjacent_reg_possible(j)];
                            pol2 = union(pol2, polyshape(PN.problems{1}.Q{adjacent_reg_possible(j)}'));
                            PN.problems{1}.fuse_centroid{end} = mean(pol2.Vertices)';
                            add_more = 1;
                        end
                    end
                end
                potential_merging_regions = setdiff(potential_merging_regions,PN.problems{1}.fuse{end});
            end
            for i = length(PN.problems{1}.fuse):-1:1
                if length(PN.problems{1}.fuse{i}) <= 1
                    PN.problems{1}.fuse(i) = [];
                    PN.problems{1}.fuse_centroid(i) = [];
                end
            end
            
            
            write_data(sprintf('%sproblemRect%d.xml',filename,PN.problems{1}.index),PN.problems{1});
            PNnew{end+1} = PN.problems{1};
            PN.problems(1)=[];

            %%%%%%%%%%% solve the problem on the refined envronment
            [PN.problems{end},flag] = solve_path_planning(PN.problems{end},m0,mf);
            PN.problems{end}.m0 = m0;
            PN.problems{end}.mf = mf;
            %PN.problems{end}.initial_prob = length(PN.problems);
            if (flag == 1) %path planning optimization problems feasible
                R0 = [];
                temp = find(PN.problems{end}.m0);
                for j = 1 : length(temp)
                    for kk = 1 : PN.problems{end}.m0(temp(j))
                        R0 = [R0 temp(j)];
                    end
                end
                [feasible_sigma, Rob_places, ~, ~] = extract_trajectories(PN.problems{end}.Pre,...
                    PN.problems{end}.Post,PN.problems{end}.m0,PN.problems{end}.sigma,R0);
                if (feasible_sigma == 1)
                    PN.problems{end}.traj = Rob_places;
                    Euclidean_distance = 0;
                    for l1 = 1 : length(Rob_places)
                        trajectRob = Rob_places{l1};
                        for l2 = 1 : length(trajectRob)-1
                            if (l2 == 1) %for the first point consider the initial point and not the centroid of the region
                                Euclidean_distance = Euclidean_distance + ...
                                    norm(PN.problems{end}.initial_points(l1,:)-PN.problems{end}.centroids{trajectRob(l2+1)});
                            elseif (l2 == length(trajectRob)-1) %for the last point consider the final point and not the centroid of the final region
                                Euclidean_distance = Euclidean_distance + ...
                                    norm(PN.problems{end}.centroids{trajectRob(l2)} - PN.problems{end}.final_points(l1,:));
                            else
                            Euclidean_distance = Euclidean_distance + ...
                                norm(PN.problems{end}.centroids{trajectRob(l2)}-PN.problems{end}.centroids{trajectRob(l2+1)});
                            end
                        end
                    end
                    PN.problems{end}.Euclidean = Euclidean_distance / length(Rob_places);
                else
                    fprintf(1,'\nNot possible to get the trajectories!');
                end
            else
                fprintf(1,'\nOptimziation problems for path planning infeasable');
            end
            %%%%%%%%%%%%%%%
        else
            write_data(sprintf('%sproblemRect%d.xml',filename,PN.problems{1}.index),PN.problems{1}); %the problem should not be refined
            PNnew{end+1} = PN.problems{1};
            PN.problems(1)=[];            
        end
    end
end
