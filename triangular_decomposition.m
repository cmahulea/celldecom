function [C,adj,obstacles]=triangular_decomposition(obstacles,env_bounds,points,edges)
%rectangular decomposition
%INPUT arguments:
%   obstacles -> a cell array with structures for each obstacle, where element obstacles{i}.vertices is a matrix with 2 rows, containing the vertices of obstacle O_i
%   env_bounds -> has form [x_min x_max y_min y_max], giving the boundaries of environment whose free space should be partitioned
%OUTPUT arguments: 
%   C -> cell array containing cells from partition, where C{i} is a matrix with 2 rows, giving the vertices of i^th cell
%   adj -> adjacency matrix (binary)
%   mid_X, mid_Y (optional) -> middle points of common segments of adjacent cells

if (nargin < 3)
    points = [];
    edges = [];
end

%first begin creating points (vertices) X
X = [env_bounds(1) env_bounds(3); %space boundary
    env_bounds(2) env_bounds(3);
    env_bounds(2) env_bounds(4);
    env_bounds(1) env_bounds(4)];

%begin creating constraints Cst (indices of lines from X giving linear constraints)
Cst = [1 2; %space boundary
         2 3;
         3 4;
         4 1];

%add obstacle information
if ~isempty(edges)
    for i=1:length(obstacles)
        ind=size(X,1)+1;    %index for points
        X=[ X; [obstacles{i}]' ];
        for j=1:(size(obstacles{i},2)-1)
            Cst=[ Cst; [ind+j-1 , ind+j ] ];  %object's vertices are arranged in order of parcurging convex hull
        end
        Cst=[ Cst; [ind+j , ind ] ];  %close current object
    end
end
%add edges information
for i=1:length(edges)
    ind=size(X,1)+1;    %index for points
    X=[ X; [edges{i}]' ];
    for j=1:(size(edges{i},2)-1)
        Cst=[ Cst; [ind+j-1 , ind+j ] ];  %object's vertices are arranged in order of parcurging convex hull
    end
    Cst=[ Cst; [ind+j , ind ] ];  %close current object
end

%constrained triangulation (Matlab 2010b); warnings may appear to indicate that constraining edges (obstacles) intersect, or some points are repeated
warning('off', 'all');
Triang = DelaunayTri([X ; points], Cst);   %special structure
warning('on', 'all');

X=Triang.X; %new points (vertices) may have appeared, in case of obstacles intersecting
triangles=Triang.Triangulation;  %indices of triangles (points from nex X)

%from the returned triangles, some are inside obstacles (obstacles are triangulated as well);
%we cannot have triangles partially overlapping with obstacles (becasue of Constrained Delaunay);
%however, new points (vertices) may appear - intersection of obstacless

%search indices of traingles from free space
ind=[]; %feasible indices of triangles (row indices in triangles)
for k=1:size(triangles,1)
    ind=[ind,k];    %assume k is good (if not, it will be removed)
    centr=mean(X(triangles(k,:),:));   %centroid of triangle k (if centroid belongs to one or more obstacles, the whole triangle belongs to that/those obstacles)
    if centr(1)<env_bounds(1) || centr(2)<env_bounds(3) || centr(1)>env_bounds(2) || centr(2)>env_bounds(4) %current triangle outside of bounds (possible if obstacles cross outside)
        ind(end)=[];    %remove k from feasible indices
        continue;  %continue with next index k
    end
    for i=1:length(obstacles) %for each obstacle
        if inpolygon(centr(1),centr(2),obstacles{i}(1,:),obstacles{i}(2,:))      %current triangle is inside obstacle i
            ind(end)=[];    %remove k from feasible indices
            break;  %continue with next index k
        end
    end
end

%construct cell C (triangle vertices in each element) and adjacency adj
%return: C- containing cel vertices (cell with each element a matrix with 2 rows and 3 columns)
%    and adj - adjacency matrix, with adj(i,j)=1 if cell i is adjacent to j (adj(i,i)=1 and adj is symmetric)

k=length(ind);  %number of feasible triangles

C=cell(1,k);
adj=sparse(eye(k)); %self transitions in every cell

for i=1:k
    C{i}=(X(triangles(ind(i),:),:))';    %in matrix triangle, the columns are indices of points from X composing the triangle
    for j=i+1:k %go only through higher indices (adjacency matrix is symmetric)
        common_v=intersect(triangles(ind(i),:), triangles(ind(j),:)); %indices of common vertices (nodes) of cells i and j
        if length(common_v)==2 %two common vertices means adjacency
            adj(i,j)=1;
            adj(j,i)=1; %symmetric
        end
    end
end
% adj=sparse(adj);    %convert to sparse

%arrange vertices of each cell from decomposition to be convex hull
for i=1:length(C)
    ch_in=convhull(C{i}(1,:),C{i}(2,:));
    C{i}=C{i}(:,ch_in(1:length(ch_in)-1));
end


if (isempty(points) || isempty(edges))
    %remove the strongly components keeping only the biggest one
    G=graph(adj);
    str_bins = conncomp(G);
    num_comp = zeros(1,max(str_bins));
    for i = 1 : length(num_comp)
        num_comp(i) = length(find(str_bins == i));
    end
    [~,I] = max(num_comp);
    elimin = [];
    for j = 1 : length(num_comp)
        if (j ~= I(1))
            elimin = [elimin find(str_bins == j)];
        end
    end
    elimin = sort(elimin);
    for j = length(elimin) : -1 : 1
        adj(elimin(j),:) = [];
        adj(:,elimin(j)) = [];
        obstacles{end+1} = C{elimin(j)};
        %    fill(C{elimin(j)}(1,:),C{elimin(j)}(2,:),'w','FaceColor','yellow','FaceAlpha',0.5);
        C(elimin(j)) = [];
    end
end