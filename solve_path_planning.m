function [problems,flag] = solve_path_planning(PN,m0,mf)
%PN - Petri net structure, m0 and mf are the initial and final markings
%variables [sigma mb], sigma is the firing count vector; mb is the number
%of time that robots cross through each region from m0 to mf

flag = 1;
[nplaces,ntrans] = size(PN.Post);
f = [ones(1,ntrans) 1000];
Aeq = [PN.C zeros(nplaces,1)];
beq = mf - m0;
A = [PN.Post -ones(nplaces,1)];
b = -m0;%zeros(nplaces,1);

options = optimset('linprog');
options.Display = 'off';
tic;
[X,~,EXITFLAG] = linprog(f,A,b,Aeq,beq,zeros(1,ntrans+1),[],options);
time_to_solve = toc;
fprintf(1,'\nSolved LP (path planning) ');
if (EXITFLAG > 0)
    bound = X(end);
    fprintf(1,'and get b=%f',bound);
    problems.congestion = ceil(bound); %for each cell, number of crossing time
    problems.time_solving_LP = time_to_solve;
    if (problems.congestion > 1) %solve a new LP since the first one could return a non-integer solution
        fprintf(1,'\nSolved a new LP (path planning) since the previous one could be not integer');
        f = ones(1,ntrans);
        Aeq = PN.C;
        beq = mf - m0;
        A = PN.Post;
        b =  problems.congestion*ones(nplaces,1)-m0;%zeros(nplaces,1);
        tic;
        [X,~,EXITFLAG] = linprog(f,A,b,Aeq,beq,zeros(1,ntrans),[],options);
        if (EXITFLAG <= 0)
            fprintf(1,'\nSecond LP cannot find a solution!');
        end
        time_to_solve2 = toc;
        problems.time_solving_LP2 = time_to_solve2;
    end
    problems.sigma = X(1:ntrans);
    problems.cell_length = sum(problems.sigma)/sum(m0);
else
    flag = 0;
    problems.error=1;
    fprintf(1,'and no Solution found!')
    return;
end
return