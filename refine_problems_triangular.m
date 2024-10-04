function PN = refine_problems_triangular(PN,filename)



%start refining until no more possible
still_refining = 1;
refined_problems = length(PN.problems);
while (still_refining == 1)
    still_refining = 0;
    %for i = 1 : length(PN.problems)
    while ~isempty(PN.problems)
        if (PN.problems{1}.should_refine == 1)
            still_refining = 1;
            fprintf(1,'\n==================================================');
            refined_problems = refined_problems + 1;
            fprintf(1,'\n Problem %d.',refined_problems);
            newPart = refine_partition_triangular(PN.problems{1});
            PN.problems{1}.should_refine = 0; %do not refining again
            if (newPart.no_refined_cells == 0)
                fprintf(1,'\nThe refined problem is not new! Skip it');
                fprintf(1,'\n==================================================');
                fprintf(1,'\n==================================================');
                fprintf(1,'\n==================================================');
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
            initial_m0 = find(PN.problems{1}.m0);
            final_mf = find(PN.problems{1}.mf);
            initial = {}; final = {};
            for j = 1 : length(initial_m0)
                for k = 1 : PN.problems{1}.m0(initial_m0(j))
                    initial{end+1} = PN.problems{1}.centroids{initial_m0(j)};
                end
            end
            for j = 1 : length(final_mf)
                for k = 1 : PN.problems{1}.mf(final_mf(j))
                    final{end+1} = PN.problems{1}.centroids{final_mf(j)};
                end
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
            write_data(sprintf(filename,PN.problems{1}.index),PN.problems{1});
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
                            Euclidean_distance = Euclidean_distance + ...
                                norm(PN.problems{end}.centroids{trajectRob(l2)}-PN.problems{end}.centroids{trajectRob(l2+1)});
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
            write_data(sprintf(filename,PN.problems{1}.index),PN.problems{1}); %the problem should not be refined
            PN.problems(1)=[];            
        end
    end
end
