function objects = generate_random_env(xmax,ymax,obs_no,obs_size,messages)

if messages
    fprintf(1,'Start generating a random environment (%d x %d).',xmax,ymax);
end
tic;
objects = random_env(xmax,ymax,obs_no,obs_size);
if messages
    fprintf(1,'\n Environment generated. Time to generate it: %f',toc)
end