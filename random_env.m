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
function objects = random_env(xmax,ymax,obs_no,obs_size)
limits = [0 xmax 0 ymax];
center=zeros(2,obs_no);
objects=cell(1,obs_no);

for i=1:obs_no 
    ax=limits(1)+obs_size;
    bx=limits(2)-obs_size;
    ay=limits(3)+obs_size;
    by=limits(4)-obs_size;    
    cx=ax+(bx-ax)*rand(1);
    cy=ay+(by-ay)*rand(1);
    center(:,i)=[cx;cy];
    
    pts=2+randi(8);
    apx=cx-obs_size;
    bpx=cx+obs_size;
    apy=cy-obs_size;
    bpy=cy+obs_size;
    
    for j=1:pts
        px=apx+(bpx-apx)*rand(1);
        py=apy+(bpy-apy)*rand(1);
        objects{i}(:,j)=[px;py];
    end
    
end

for i = 1 : length(objects)
    k = convhull(objects{i}');
    objects{i}=objects{i}(:,k(1:length(k)-1));
end
return


