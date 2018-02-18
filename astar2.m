function [route] = astar2(exbigraph,exbiloc,startnode,endnode)

graph=exbigraph;
Loc=exbiloc;
n=size(graph,1);
Cost=inf(n,1);

curr=startnode;
Route=cell(n,1);
Cost(curr)=0;
Tabu=[curr];
cRoute=[curr];


% pre computed heuristic dist
for i=1:n
    HCost(i,1)=costcal(Loc(endnode,:),Loc(i,:));
end

i=1;

while i<n & curr ~=endnode    
    i=i+1;
    T=1:n;
    T(Tabu)=[];    
    I=find(graph(curr,T)==1);
    cost=costcal(Loc(curr,:),Loc(T(I),:)) + Cost(curr);
    J=find(Cost(T(I))>cost);
    Cost(T(I(J)))=cost(J);    
    for j=1:length(J)
        Route{T(I(J(j)))}=[cRoute T(I(J(j)))];        
    end  
    
    [mn cr]=min(Cost(T)+HCost(T));
    if isinf(mn)
        i=n;
    end
    curr=T(cr);
    cRoute=Route{curr};
    Tabu=[Tabu curr];
end
if curr ==endnode
    route=cRoute;
else
    route=[];
end

