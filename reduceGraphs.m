function [Graphs,dataset] = reduceGraphs(PNpr,filename)

max_l = length(PNpr);
for i = 1 : max_l
    fprintf(1,'\nCompute graph %d out of %d.',i,max_l);
    Graphs{i}.states = {};
    Graphs{i}.centroid = {};
    Graphs{i}.C = {};
    if isfield(PNpr{i},'fuse')
        fused_states = cell2mat(PNpr{i}.fuse);
    else
        fused_states = [];
    end
    for j = 1 : length(PNpr{i}.Q)
        if ~any(fused_states(:) == j) %this state is not fused
            Graphs{i}.states{end+1} = j;
            Graphs{i}.centroid{end+1} = PNpr{i}.centroids{j};
            Graphs{i}.C{end+1} = PNpr{i}.Q{j};
        else
            for ll = 1 : length(PNpr{i}.fuse)
                if any(PNpr{i}.fuse{ll}(:) == j)
                    regions = PNpr{i}.fuse{ll};
                    Graphs{i}.states{end+1} = regions;

                    pol = polyshape();
                    for k = 1 : length(regions)
                        pol = union(pol,polyshape(PNpr{i}.Q{regions(k)}'));
                    end
                    Graphs{i}.C{end+1} = pol.Vertices';


                    Graphs{i}.centroid{end+1} = PNpr{i}.fuse_centroid{ll};
                    PNpr{i}.fuse{ll} = [];
                end
            end
        end
    end
    %create the adjancency matrix
    Graphs{i}.adj = sparse(length(Graphs{i}.states),length(Graphs{i}.states));
    for j = 1 : length(Graphs{i}.states)
        original_states = Graphs{i}.states{j};
        adjancency_original = [];
        for k = 1 : length(original_states)
            adjancency_original = [adjancency_original find(PNpr{i}.adj(original_states(k),:))];
        end
        adjancency_original = setdiff(adjancency_original,original_states);
        adjancency_original = unique(adjancency_original,'sorted');
        for k = 1 : length(adjancency_original)
            original_region = adjancency_original(k);
            for ll = 1 : length(Graphs{i}.states)
                if any(Graphs{i}.states{ll} == original_region)
                    Graphs{i}.adj(j,ll) = 1;
                    Graphs{i}.adj(ll,j) = 1;
                    break;
                end
            end
        end
    end
    Graphs{i}.initial_points = PNpr{i}.initial_points;
    Graphs{i}.final_points = PNpr{i}.final_points;
    Graphs{i}.obstacles = PNpr{i}.obstacles;
    Graphs{i}.limits = PNpr{i}.limits;
    Graphs{i}.index = PNpr{i}.index;
    Graphs{i}.refined = PNpr{i}.indices_refined;

    write_data_graph(sprintf('%sproblemGraph%d.xml',filename,Graphs{i}.index),Graphs{i});

    dataset(i).index = Graphs{i}.index;
    dataset(i).refined = Graphs{i}.refined;
    %create the environment. Eacjh element is equal with the cell nmber to
    %which belong or 0 if it is an obstacle
    dataset(i).environment=zeros(32,32);
    for j = 0 : 31
        for k = 0 : 31
            for ll = 1 : length(Graphs{i}.C)
                % Extraer las coordenadas mínimas y máximas en x e y de la
                % celda ll
                x_min_grande = min(Graphs{i}.C{ll}(1,:));
                x_max_grande = max(Graphs{i}.C{ll}(1,:));
                y_min_grande = min(Graphs{i}.C{ll}(2,:));
                y_max_grande = max(Graphs{i}.C{ll}(2,:));
                % Calcular las coordenadas de los vértices del cuadrado más pequeño
                vertices_pequeno = [
                    j, k;                     % Vértice inferior izquierdo
                    j+1, k;         % Vértice inferior derecho
                    j+1, k+1; % Vértice superior derecho
                    j, k+1          % Vértice superior izquierdo
                    ];
                % Comprobar si todos los vértices del cuadrado pequeño están dentro del cuadrado grande
                if all(vertices_pequeno(:, 1) >= x_min_grande) && all(vertices_pequeno(:, 1) <= x_max_grande) && ...
                        all(vertices_pequeno(:, 2) >= y_min_grande) && all(vertices_pequeno(:, 2) <= y_max_grande)
                    dataset(i).environment(j+1,k+1) = ll;
                    break;
                end
            end
        end
    end
    dataset(i).start = zeros(5,2);
    for j = 1 : size(Graphs{i}.initial_points,1)
        robot = Graphs{i}.initial_points(j,:);
        dataset(i).start(j,1) = ceil(robot(1));
        dataset(i).start(j,2) = ceil(robot(2));
    end
    dataset(i).end = zeros(5,2);
    for j = 1 : size(Graphs{i}.final_points,1)
        robot = Graphs{i}.final_points(j,:);
        dataset(i).end(j,1) = ceil(robot(1));
        dataset(i).end(j,2) = ceil(robot(2));
    end
    dataset(i).num_cells = length(Graphs{i}.C);
    dataset(i).distance = 0;
    dataset(i).congestion = PNpr{i}.congestion;

    trajectories = PNpr{i}.traj;  % Obtener las trayectorias del conjunto actual
    numTrajectories = length(trajectories);  % Número de trayectorias
    traject = cell(1, numTrajectories);  % Preasignar una celda para almacenar las trayectorias

    for j = 1:numTrajectories
        traj = trajectories{j};  % Obtener la trayectoria actual
        numPoints = length(traj);  % Número de puntos en la trayectoria
        currentTraject = zeros(numPoints , 2);  % Preasignar espacio para la trayectoria

        % Colocar los centroides del resto de la trayectoria (excepto el último punto)
        for k = 1:numPoints
            currentTraject(k, :) = PNpr{i}.centroids{traj(k)};
            if (k > 1)
                dataset(i).distance = dataset(i).distance + norm(currentTraject(k-1,:)-currentTraject(k,:));
            end
        end
        % Guardar la trayectoria completa en la celda
        traject{j} = currentTraject;
    end

    dataset(i).traj = traject;
end
fprintf(1,'\n');
