%    This is part of RMTool - Robot Motion Toolbox, for Matlab 2010b or newer.
%
%    Copyright (C) 2016 RMTool developing team. For people, details and citing 
%    information, please see: http://webdiis.unizar.es/RMTool/index.html.
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.

%% ============================================================================
%   MOBILE ROBOT TOOLBOX
%   Graphical User Interface
%   First version released on September, 2014. 
%   Last modification February 19, 2018.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

% Function to create Random Enviroment
function objects = random_env(obs_no,obs_size)
limits = axis;
hold on
grid on

center=zeros(2,obs_no);
objects=cell(1,obs_no);

%% Obstacles

for i=1:obs_no 
    ax=limits(1)+obs_size;
    bx=limits(2)-obs_size;
    ay=limits(3)+obs_size;
    by=limits(4)-obs_size;    
    cx=ax+(bx-ax)*rand(1);
    cy=ay+(by-ay)*rand(1);
    center(:,i)=[cx;cy];
    
    pts=2+randi(20);
    apx=cx-obs_size;
    bpx=cx+obs_size;
    apy=cy-obs_size;
    bpy=cy+obs_size;
    
    apxb=apx-limits;
    bpxb=bpx+limits;
    apyb=apy-limits;
    bpyb=bpy+limits;
    
 %   objects_bound{i}=[apxb bpxb bpxb apxb; apyb apyb bpyb bpyb];

    for j=1:pts
        px=apx+(bpx-apx)*rand(1);
        py=apy+(bpy-apy)*rand(1);
        objects{i}(:,j)=[px;py];
    end
    
    %creating convex obstacles & drawing them
    k=convhull(objects{i}(1,:),objects{i}(2,:));
    objects{i}=polyshape(objects{i}(:,k(1:length(k)-1))');
    plot(objects{i},'FaceColor','black');
end

return


%% Initial and final points

for k=1:rob_no %Initial points
    not=1;
    while not>=1
        not=0;
        sx=abs(limits(1)+(limits(2)-limits(1))*rand(1)-rob_size);
        sy=(limits(3)+(limits(4)-limits(3))*rand(1))/10; %initial points in the 10% on bottom part of the environment
        initial_point{k}=rob_size*round([sx,sy]/rob_size);
        initial_polyshape{k} = polyshape([initial_point{k}(1) initial_point{k}(1)+rob_size ...
            initial_point{k}(1)+rob_size,initial_point{k}(1)],[initial_point{k}(2) initial_point{k}(2) ...
            initial_point{k}(2)+rob_size initial_point{k}(2)+rob_size]);
        %Objetos
        for o=1:length(objects)
            if (area(intersect(objects{o},initial_polyshape{k}))>1000*eps)
                not=not+1;
            end  
        end   
        
        %Puntos iniciales
        if k>1
            for t=1:(k-1)
                if (area(intersect(initial_polyshape{t},initial_polyshape{k}))>1000*eps)
                    not=not+1;
                end      
            end
        end 
        
    end
end

for k=1:dest_no %Final points
    not=1;
    while not>=1
        not=0;
        dx=abs(limits(1)+(limits(2)-limits(1))*rand(1)-rob_size);
        dy=limits(4)/2-rob_size+(limits(4)/2-limits(3))*rand(1);
        final_point{k}=rob_size*round([dx,dy]/rob_size);
        final_polyshape{k} = polyshape([final_point{k}(1) final_point{k}(1)+rob_size ...
            final_point{k}(1)+rob_size,final_point{k}(1)],[final_point{k}(2) final_point{k}(2) ...
            final_point{k}(2)+rob_size final_point{k}(2)+rob_size]);


        %Objetos
        for o=1:length(objects)
            if area(intersect(objects{o},final_polyshape{k}))>1000*eps
                not=not+1;
            end
        end   


        %Puntos iniciales
        for t=1:length(initial_point)
            if area(intersect(initial_polyshape{t},final_polyshape{k}))>1000*eps
                not=not+1;
            end
        end
        
        %Puntos finales
        if k>1
            for t=1:(k-1)
                if area(intersect(final_polyshape{t},final_polyshape{k}))>1000*eps
                    not=not+1;
                end
            end
        end
        
    end  
end
   

%% Plotting points

for i=1:length(initial_point)   
    plot(initial_polyshape{i},'FaceColor','red');
%    text((initial_point{i}(1)+0.2),(initial_point{i}(2)+0.2),{num2str(i)});
    hold on;
end

pause(0.1)
for i=1:length(final_point)
    plot(final_polyshape{i},'FaceColor','green');
    hold on;
end
pause(0.1)

end




