clear all;
xmax = 32;
ymax = 32;
obs_no = 100;
obs_size = 1;
dest_no = 5;
rob_no = 4;
simul_no = 100;
rob_size = 1; %square robot rob_size x rob_size

env_OK = 0;
while ~env_OK
    figure;axis([0 xmax 0 ymax]);

    fprintf(1,'Start generating a random environment (%d x %d).',xmax,ymax);
    tic;
    objects = random_env(obs_no,obs_size);
    fprintf(1,'\n Environment generated. Time to generate it: %f',toc)

    fprintf(1,'\n\n Start partioning the environment with rectangular cell decomposition.');
    tic;

    for i = 1 : length(objects)
        objectsV{i}.vertices = objects{i}.Vertices';
    end
    [C,adj,varargout] = rectangular_decomposition(objectsV,[0, xmax, 0, ymax]);%,10*round(max(xmax,ymax)/rob_size));
    fprintf(1,'\n Cell decomposition completed. Time to compute it: %f',toc)

    %represent cells:
    for i=1:length(C)
        fill(C{i}(1,:),C{i}(2,:),'w','EdgeColor',[.8 .8 .8],'FaceAlpha',0.5);
    end
    centroids = cell(1,length(C));
    for i=1:length(C)
        centr=mean(C{i},2)';
        text(centr(1),centr(2),sprintf('p_{%d}',i),'HorizontalAlignment','center','Color','k','FontSize',10,'FontAngle','italic','FontName','TimesNewRoman');
        centroids{i} = centr;
    end

    pause(0.01);
    fprintf(1,'\n\n Start constructing the Petri net.');
    tic;
    PN = construct_PN(C,adj,ymax);
    fprintf(1,'\n Petri net constructed. Time to compute it: %f\n',toc)
    initial_cells = PN.potential_m0;
    for i=1:length(initial_cells)
        fill(C{initial_cells(i)}(1,:),C{initial_cells(i)}(2,:),'w','FaceColor','red','FaceAlpha',0.5);
    end
    final_cells = PN.potential_mf;
    for i=1:length(final_cells)
        fill(C{final_cells(i)}(1,:),C{final_cells(i)}(2,:),'w','FaceColor','green','FaceAlpha',0.5);
    end
    if (isempty(initial_cells) || isempty(final_cells))
        fprintf(1,"\nGenerate again the environment since no possible to have initial or final cells!");
    else
        env_OK = 1;
    end
end
save environment_3;

%load environment_2;

PN.Q = C;
PN.adj = adj;
PN.centroids = centroids;
PN.obstacles = objectsV;
PN.limits = [xmax,ymax];
number_of_markings = min(rob_no,length(PN.potential_m0));
temp = min(dest_no,length(PN.potential_mf));
number_of_markings = min(number_of_markings,temp);

PN.problems = cell(1,simul_no);
for k = 1 : simul_no
    rand_m0 = randi(length(PN.potential_m0),number_of_markings,1);
    rand_mf = randi(length(PN.potential_mf),number_of_markings,1);
    m0 = sparse(size(PN.Pre,1),1);
    mf = m0;
    for ll = 1 : length(rand_m0)
        m0(PN.potential_m0(rand_m0(ll))) = m0(PN.potential_m0(rand_m0(ll))) + 1;
    end
    for ll = 1 : length(rand_mf)
        mf(PN.potential_mf(rand_mf(ll))) = mf(PN.potential_mf(rand_mf(ll))) + 1;
    end
    fprintf(1,'\n==================================================');
    fprintf(1,'\nProblem %d.',k);
    [PN.problems{k},flag] = solve_path_planning(PN,m0,mf);
    PN.problems{k}.m0 = m0;
    PN.problems{k}.mf = mf;   
    if (flag == 1) %path planning optimization problems feasible
        R0 = [];
        temp = find(PN.problems{k}.m0);
        for j = 1 : length(temp)
            for kk = 1 : PN.problems{k}.m0(temp(j))
                R0 = [R0 temp(j)];
            end
        end
        [feasible_sigma, Rob_places, Rob_trans, Rob_positions] = extract_trajectories(PN.Pre,...
            PN.Post,PN.problems{k}.m0,PN.problems{k}.sigma,R0);
        if (feasible_sigma == 1) 
            PN.problems{k}.traj = Rob_places;
            Euclidean_distance = 0;
            for l1 = 1 : length(Rob_places)
                trajectRob = Rob_places{l1};
                for l2 = 1 : length(trajectRob)-1
                    Euclidean_distance = Euclidean_distance + ...
                        norm(PN.centroids{trajectRob(l2)}-PN.centroids{trajectRob(l2+1)});
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
fprintf(1,'\nStart refining ');

new = refine_partition(PN);

for i = 1 : length(new)
%    figure; hold on;
    PNnew{i} = construct_PN(new{i}.Q,new{i}.adj,ymax);
    PNnew{i}.Q = new{i}.Q;
    PNnew{i}.adj = new{i}.adj;
    PNnew{i}.obstacles = new{i}.obstacles;
    PNnew{i}.centroids = new{i}.centroids;
    PNnew{i}.limits = new{i}.limits;
    fprintf(1,'\n Petri net constructed. Time to compute it: %f\n',toc)
    %represnet obstacles
    % for j = 1 : length(PNnew{i}.obstacles)
    %     plot(polyshape(PNnew{i}.obstacles{j}.vertices'));
    % end
    % %represent cells:
    % for j=1:length(PNnew{i}.Q)
    %     fill(PNnew{i}.Q{j}(1,:),PNnew{i}.Q{j}(2,:),'w','EdgeColor',[.8 .8 .8],'FaceAlpha',0.5);
    % end
    % for j=1:length(PNnew{i}.Q)
    %     centr=mean(PNnew{i}.Q{j},2)';
    %     text(centr(1),centr(2),sprintf('p_{%d}',j),'HorizontalAlignment','center','Color','k','FontSize',10,'FontAngle','italic','FontName','TimesNewRoman');
    % end
    % initial_cells = PNnew{i}.potential_m0;
    % for j=1:length(initial_cells)
    %     fill(PNnew{i}.Q{initial_cells(j)}(1,:),PNnew{i}.Q{initial_cells(j)}(2,:),'w','FaceColor','red','FaceAlpha',0.5);
    % end
    % final_cells = PNnew{i}.potential_mf;
    % for j=1:length(final_cells)
    %     fill(PNnew{i}.Q{final_cells(j)}(1,:),PNnew{i}.Q{final_cells(j)}(2,:),'w','FaceColor','green','FaceAlpha',0.5);
    % end
    initial_m0 = find(PN.problems{i}.m0);
    final_mf = find(PN.problems{i}.mf);
    initial = {}; final = {};
    for j = 1 : length(initial_m0)
        for k = 1 : PN.problems{i}.m0(initial_m0(j))
            initial{end+1} = PN.centroids{initial_m0(j)};
        end
    end
    for j = 1 : length(final_mf)
        for k = 1 : PN.problems{i}.mf(final_mf(j))
            final{end+1} = PN.centroids{final_mf(j)};
        end
    end
    m0 = sparse(length(PNnew{i}.Q),1);
    mf = sparse(length(PNnew{i}.Q),1);
    for j = 1 : length(initial)
        for k = 1 : length(PNnew{i}.Q)
            [in,on] = inpolygon(initial{j}(1),initial{j}(2),PNnew{i}.Q{k}(1,:),PNnew{i}.Q{k}(2,:));
            if (in+on>=1)
                m0(k) = m0(k) + 1;
                break;
            end
        end
    end
    for j = 1 : length(final)
        for k = 1 : length(PNnew{i}.Q)
            [in,on] = inpolygon(final{j}(1),final{j}(2),PNnew{i}.Q{k}(1,:),PNnew{i}.Q{k}(2,:));
            if (in+on>=1)
                mf(k) = mf(k) + 1;
                break;
            end
        end
    end
    %%%%%%%%%%% solve the problem on the refined envronment
    fprintf(1,'\n==================================================');
    fprintf(1,'\nRefined Problem %d.',i);
    [PNnew{i}.problem,flag] = solve_path_planning(PNnew{i},m0,mf);
    PNnew{i}.m0 = m0;
    PNnew{i}.mf = mf;
    PNnew{i}.initial_prob = i;
    if (flag == 1) %path planning optimization problems feasible
        R0 = [];
        temp = find(PNnew{i}.m0);
        for j = 1 : length(temp)
            for kk = 1 : PNnew{i}.m0(temp(j))
                R0 = [R0 temp(j)];
            end
        end
        [feasible_sigma, Rob_places, Rob_trans, Rob_positions] = extract_trajectories(PNnew{i}.Pre,...
            PNnew{i}.Post,PNnew{i}.m0,PNnew{i}.problem.sigma,R0);
        if (feasible_sigma == 1)
            PNnew{i}.traj = Rob_places;
            Euclidean_distance = 0;
            for l1 = 1 : length(Rob_places)
                trajectRob = Rob_places{l1};
                for l2 = 1 : length(trajectRob)-1
                    Euclidean_distance = Euclidean_distance + ...
                        norm(PNnew{i}.centroids{trajectRob(l2)}-PNnew{i}.centroids{trajectRob(l2+1)});
                end
            end
            PNnew{i}.problem.Euclidean = Euclidean_distance / length(Rob_places);
        else
            fprintf(1,'\nNot possible to get the trajectories!');
        end
    else
        fprintf(1,'\nOptimziation problems for path planning infeasable');
    end


    %%%%%%%%%%%%%%%
end
fprintf(1,'\n');
