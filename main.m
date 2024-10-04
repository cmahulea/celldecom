clear all;
xmax = 32;
ymax = 32;
obs_no = 100;
obs_size = 1;
dest_no = 5;
rob_no = 4;
simul_no = 100;
rob_size = 1; %square robot rob_size x rob_size
save_data = '/home/cosmos/cmahulea/matlab_condor/celldecom/data';
%save_data = '.\\data\\';
env_OK = 0;
while ~env_OK
    env_OK = 1;
    objects = generate_random_env(xmax,ymax,obs_no,obs_size,1);
    PN = partition_env(xmax,ymax,objects,'rectangular',1); 
 %   PN1 = partition_env(xmax,ymax,objects,'triangular',1);    
    if (isempty(PN.potential_m0) ||  isempty(PN.potential_mf) )
        fprintf(1,'\n=======================================================================');
        fprintf(1,'\n===========ENVIRONMENT NOT WELL DEFINED================================');
        fprintf(1,'\n=======================================================================\n');
        env_OK = 0;
    end
end

number_of_markings = min(rob_no,length(PN.potential_m0));
number_of_markings = min(number_of_markings,min(dest_no,length(PN.potential_mf)));


PN = generate_random_problems(PN,simul_no,number_of_markings); %generate random problems 

fprintf(1,'\nStart refining using rectangular cell decomposition...');
PNpr = refine_problems_rectangular(PN,sprintf('%sproblemRect%%d.xml',save_data));
fprintf(1,'\nFinished refining.');
fprintf(1,'\n');

%save environment_3;
%load environment_3;

Graphs = reduceGraphs(PNpr,sprintf('%sproblemGraph%%d.xml',save_data));

%PN1 = generate_random_problems(PN1,simul_no,number_of_markings); %generate random problems for rectangular cell decomposition
fprintf(1,'\nStart refining using triangular cell decomposition...');
%PN1 = refine_problems_triangular(PN1,'.\\data\\problemTri%d.xml');
fprintf(1,'\nFinished refining.');
fprintf(1,'\n');
