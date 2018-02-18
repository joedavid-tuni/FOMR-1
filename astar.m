function [route, cs] = astar(exbigraph,exbiloc,startnode,endnode)
% calculate route of given graph start and end node.

nn=size(exbigraph,1);
sn=startnode;
en=endnode;
loc=exbiloc;
Route=[sn];
Ccost=[0];
Hcost=[0];
crot=[];
curr=sn;

cst=[];
rta=[];

ind=1;
mn=0;
status=0;
% if we have not found endnode yet go ahead
while Route(ind,1)~=en & size(Route,1)>0 % while the end node become minimum cost node
    
    tmp=Route(ind,:);
    tmp(tmp==0)=[];
    curr=tmp(1);
    crot=tmp;
    % select all nodes except the nodes  which are on our current route
    T=1:nn;
    T(crot)=[];
    % find connected node from the current node
    I=find(exbigraph(curr,T)==1);
    Route(ind,:)=[];
    excost=Ccost(ind,:);
    Ccost(ind,:)=[];
    Hcost(ind,:)=[];
    % if current node has connection
    if ~isempty(I)
        % take the connected nodes routes
        rtmat=[T(I)' repmat(crot,length(I),1)];
        s1=size(Route,2);
        s2=size(rtmat,2);
        Route(:,s1+1:s2)=0;
        rtmat(:,s2+1:s1)=0;
        % calculate heuristic cost of connected nodes
        hcost=costcal(loc(en,:),loc(T(I),:));
        % calculate past cost plus distance from current node to connected
        % one
        ccost=excost+costcal(loc(curr,:),loc(T(I),:));
        % for all the nodes which are connected to current one
        for i=1:size(rtmat,1)
            % if connected one route is our already visited rote than chach
            % which ones cost is minimum then update route and cost
            I=find(Route(:,1)==rtmat(i,1));
            if ~isempty(I)
                if Ccost(I(1))>ccost(i)
                    Ccost(I(1))=ccost(i);
                    Hcost(I(1))=hcost(i);
                    Route(I(1),:)=rtmat(i,:);
                end
            else
                % if new connected nodes route is not in our list then add
                % those
                Route=[Route;rtmat(i,:)];
                Ccost=[Ccost;ccost(i)];
                Hcost=[Hcost;hcost(i)];                
            end
        end
    end
    
    [mn ind]=min(Ccost+Hcost);    
    
end
if size(Route,1)>0
    route=Route(ind,:);
    route(route==0)=[];
    cs=mn;
else
    route=[];
    cs=inf;
end













