function PN = construct_PN(C,adj,ymax)

%compute the Petri net model
nplaces=length(C); %number of places
ntrans = sum(sum(full(adj))) - nplaces; %number of transitions
Pre = sparse(nplaces,ntrans);
Post = sparse(nplaces,ntrans);
%construct matrices Pre and Post
ntrans_actual = 0;
for i=1:nplaces-1
    for j=i+1:nplaces
        if adj(i,j)==1
            ntrans_actual=ntrans_actual+2;
            Pre(i,ntrans_actual-1)=1;
            Pre(j,ntrans_actual)=1;
            Post(i,ntrans_actual)=1;
            Post(j,ntrans_actual-1)=1;
        end
    end
end
PN.Pre = Pre;
PN.Post = Post;
PN.C=Post-Pre;
PN.nplaces = nplaces;
PN.ntrans = ntrans;

%compute the potential initial marking
PN.potential_m0 = [];
for k=1:length(C) %Initial points
    centr = mean(C{k}');
    if (centr(2) < ymax*0.3)  %the cell is in the 40% of the bottom of the environment
        PN.potential_m0 = [PN.potential_m0 k];
    end
end

PN.potential_mf = [];
for k=1:length(C) %destination points
    centr = mean(C{k}');
    if (centr(2) >= ymax*0.6) %the destinations will be in the 50% of the top of the environment
        PN.potential_mf = [PN.potential_mf k];
    end
end
return
