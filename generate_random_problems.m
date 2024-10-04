function PN = generate_random_problems(PN,simul_no,number_of_markings)


PN.problems = cell(1,simul_no);
for k = 1 : simul_no
    rand_m0 = randi(length(PN.potential_m0),number_of_markings,1);
    rand_mf = randi(length(PN.potential_mf),number_of_markings,1);
    m0 = sparse(size(PN.Pre,1),1);
    mf = m0;
    initial_points = [];
    for ll = 1 : length(rand_m0)
        m0(PN.potential_m0(rand_m0(ll))) = m0(PN.potential_m0(rand_m0(ll))) + 1;
        initial_points = [initial_points; 0.1+PN.centroids{PN.potential_m0(rand_m0(ll))}];
    end
    final_points = [];
    for ll = 1 : length(rand_mf)
        mf(PN.potential_mf(rand_mf(ll))) = mf(PN.potential_mf(rand_mf(ll))) + 1;
        final_points = [final_points; 0.1+PN.centroids{PN.potential_mf(rand_mf(ll))}];
    end
    fprintf(1,'\n==================================================');
    fprintf(1,'\nProblem %d.',k);
    [PN.problems{k},flag] = solve_path_planning(PN,m0,mf);
    PN.problems{k}.index = k;
    PN.problems{k}.Q = PN.Q;
    PN.problems{k}.adj = PN.adj;
    PN.problems{k}.centroids = PN.centroids;
    PN.problems{k}.obstacles = PN.obstacles;
    PN.problems{k}.limits = PN.limits;
    PN.problems{k}.indices_refined = [];
    PN.problems{k}.Pre = PN.Pre;
    PN.problems{k}.Post = PN.Post;
    PN.problems{k}.C = PN.C;
    PN.problems{k}.m0 = m0;
    PN.problems{k}.mf = mf;   
    PN.problems{k}.initial_points = initial_points;
    PN.problems{k}.final_points = final_points;
    PN.problems{k}.should_refine = 1;
    if (flag == 1) %path planning optimization problems feasible
        R0 = [];
        temp = find(PN.problems{k}.m0);
        for j = 1 : length(temp)
            for kk = 1 : PN.problems{k}.m0(temp(j))
                R0 = [R0 temp(j)];
            end
        end
        [feasible_sigma, Rob_places, ~, ~] = extract_trajectories(PN.Pre,...
            PN.Post,PN.problems{k}.m0,PN.problems{k}.sigma,R0);
        if (feasible_sigma == 1) 
            PN.problems{k}.traj = Rob_places;
            Euclidean_distance = 0;
            for l1 = 1 : length(Rob_places)
                trajectRob = Rob_places{l1};
                for l2 = 1 : length(trajectRob)-1
                    if (l2 == 1) %for the first point consider the initial point and not the centroid of the region
                        Euclidean_distance = Euclidean_distance + ...
                            norm(PN.problems{k}.initial_points(l1,:)-PN.centroids{trajectRob(l2+1)});
                    elseif (l2 == length(trajectRob)-1) %for the last point consider the final point and not the centroid of the final region
                        Euclidean_distance = Euclidean_distance + ...
                            norm(PN.centroids{trajectRob(l2)} - PN.problems{k}.final_points(l1,:));
                    else
                        Euclidean_distance = Euclidean_distance + ...
                            norm(PN.centroids{trajectRob(l2)}-PN.centroids{trajectRob(l2+1)});
                    end
                end
            end
            PN.problems{k}.Euclidean = Euclidean_distance / length(Rob_places);
        else
            fprintf(1,'\nNot possible to get the trajectories!');
        end
    else
        fprintf(1,'\nOptimziation problems for path planning infeasable');
    end
end
