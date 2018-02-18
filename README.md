# Trajectory Generation and Path Planning of Mobile Robots

## Description of the Task
1. A map is given in a vector representation for an area of 30x30 units. Construct a graph (a topological map) by using the method of probabilistic roadmap so that there are at least 100 locations. 

<a href="https://drive.google.com/uc?export=view&id=1kgzgP-aFuqD5N0w2gdE5i3HiP5BKE_R_"><img src="https://drive.google.com/uc?export=view&id=1kgzgP-aFuqD5N0w2gdE5i3HiP5BKE_R_" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

2. Write algorithms for finding the best route, i.e. the one that minimizes cost of the route, from (5, 29, 0) to (29, 20, 0). The cost of turning 1 radian is equivalent to the cost of travelling a distance of 1 unit.
Algorithms for the three methods:
* Dijkstra
* A*
* Dynamic programming

3. Demonstrate your A* algorithm with Gazebo Turtlebot simulator.
* You can use an empty Gazebo world and an empty map
* ROS package move_base can be used to give goal locations to the robot
* visualize the trajectory taken by the robot in Matlab

4.  Assume that in the same area another robot is moving. You know the route this robot is taking (i.e. at which location or edge it is at each time instant). The robots may not be in the same location, nor are they allowed to travel the same edge in opposite directions. The robots are assumed moving synchronously: in one time instant, each robot first turns and then
moves between two nodes connected with a single edge. A robot may choose to stay at one location for a time instant. Write an algorithm for finding the optimal path for your robot, given the route for the other robot.

5. With large number of robots things get complicated. Study the article: Yu, LaValle, “Optimal multirobot path planning on graphs: complete algorithms and effective heuristics”, IEEE Transactions on Robotics, 32, 1163-1177 (2016)

## Programming Environment
* Language: Matlab Script
* IDE: Matlab
* Framework: ROS, Ubuntu

----------------------------------------------------------------------------------------------------------------------------------------


# Solution

Only Main Functions are shown here. For the rest of the functions check this github repsitory

## Generation of directed and undirected graphs

<a href="https://drive.google.com/uc?export=view&id=11HrklDw6U8ZA9I19ijUVGJ2nK29cwNVp"><img src="https://drive.google.com/uc?export=view&id=11HrklDw6U8ZA9I19ijUVGJ2nK29cwNVp" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>


<a href="https://drive.google.com/uc?export=view&id=12kg4kLkcTl5uZ4dl8khHsfHBbJPx1jSb"><img src="https://drive.google.com/uc?export=view&id=12kg4kLkcTl5uZ4dl8khHsfHBbJPx1jSb" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>


### undirected graph

```matlab
function [undirectedGraph,unedges]=generate_undirected_graph(map,nodelocation)
% takes map and produce undirected map and its adge set

% take number of node
nnode=size(nodelocation,1);
% set graph and edges as zero
undirectedGraph=zeros(nnode,nnode);
unedges=[];

%generate undirected graph

for i=1:nnode-1 
    for j=i+1:nnode
        % take each pair of node^s location x and y
        x1=[nodelocation(i,1);nodelocation(j,1)];
        y1=[nodelocation(i,2);nodelocation(j,2)];  
        % check this two nodes intersect any obstacle or not
        [xi,yi] = polyxpoly(x1,y1,map.obsx,map.obsy);
        % if there is not any intersection
        if length(xi)==0
            % connect this two nodes in graph
            undirectedGraph(i,j)=1;
            undirectedGraph(j,i)=1;
            % add this connection to adge set
            unedges=[unedges;i j;j i];
        end
    end
end
% plot graph
hold on;
for i=1:2:size(unedges,1)
    x1=[nodelocation(unedges(i,1),1);nodelocation(unedges(i,2),1)];
    y1=[nodelocation(unedges(i,1),2);nodelocation(unedges(i,2),2)]; 
    line(x1,y1);
end
hold off;
```

### directed graph

```matlab
function [bigraph,edges,nodIndex,biloc]=generate_directed_graph(undirectedGraph,nodelocation)
% this function takes undirected graph and location 
% then generate directed graph its edges its index which shows which new
% nodes takes places in the same location on orginal nodes
% and biloc all nodes location in directioned graph


edges=[];
nodIndex=[];
N=size(undirectedGraph,1);


% generate subgraph for each node
n=0;
for i=1:N   
    % take one node's all connection in undirected graph
    nodecon{i}=undirectedGraph(i,:);
    % produce its subgraph
    [subgraph{i} Conn{i} subedge{i}]=generate_subgraph(nodecon{i});  
    n=n+length(Conn{i});
    ind=i*ones(length(Conn{i}),1);
    % set its index
    nodIndex=[nodIndex;ind];
end

% concate all subgraph to one graph
bigraph=zeros(n,n);
biloc=zeros(n,3);

% set all subgraph on diagonal 
i=0;
for j=1:N
    st1(j)=i;
    st2(j)=i;
    m=size(subgraph{j},1);
    bigraph(i+1:i+m,i+1:i+m)=subgraph{j};       
    edges=[edges;subedge{j}+i];
    i=i+m;
end
% set connection of inter-subgraph's 
% all subgraph has even number of connection.
% odd ones, 1,3,5 are incoming nodes,
% even connection 2,4,.. are outgoings node
for i=1:N
    I=Conn{i}(1:2:end);
    for j=1:length(I)
        %  inter subgrpah connections establish 
        % one's outgoing connect to anothers incoming node
        i1=st1(i)+2*j;
        i2=st2(I(j))+1;
        bigraph(i1,i2)=1;
        edges=[edges;i1 i2];
        % calculate angle of node
        ang=atan2( nodelocation(nodIndex(i2),2)-nodelocation(nodIndex(i1),2),nodelocation(nodIndex(i2),1)-nodelocation(nodIndex(i1),1));
        % set location x,y and angle of node
        biloc(i1,:)=[nodelocation(nodIndex(i1),1:2) ang]; 
        biloc(i2,:)=[nodelocation(nodIndex(i2),1:2) ang]; 
        st2(I(j))=st2(I(j))+2;
    end
end
```

## Path Planning Algorithms

### A* Algorithm

<a href="https://drive.google.com/uc?export=view&id=1kgG-UoUCTqANTEjnGuShzQ1SLgdjBqd"><img src="https://drive.google.com/uc?export=view&id=1kgG-UoUCTqANTEjnGuShzQ1SLgdjBqd" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

```matlab
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

```
### Dijkstra Algorithm

<a href="https://drive.google.com/uc?export=view&id=1tpdrciWh8wgrT_9hrtIFVHNoR3coiuCK"><img src="https://drive.google.com/uc?export=view&id=1tpdrciWh8wgrT_9hrtIFVHNoR3coiuCK" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

```matlab
function [Route, Cost] = dijkstra(exbigraph,exbiloc,startnode)
% calculate route for given graph and starting point.
% this function finds all targets so you do not need to support end point

graph=exbigraph;
Loc=exbiloc;
n=size(graph,1);
Cost=inf(n,1);
curr=startnode;
Route=cell(n,1);
Cost(curr)=0;
Tabu=[curr];
cRoute=[curr];
i=1;
% for number of nodes
while i<n
    i=i+1;
    % take all possible node list
    T=1:n;
    % delete already visited node
    T(Tabu)=[];   
    % find connected node from current node which are not visited yet
    I=find(graph(curr,T)==1);
    % calculate connected nodes euclid distance from current node and add
    % current nodes cost
    cost=costcal(Loc(curr,:),Loc(T(I),:)) + Cost(curr);
    % if new finding costs is less than old cost then replace
    J=find(Cost(T(I))>cost);
    Cost(T(I(J)))=cost(J);   
    % update new comming node rote if the new cost is more efficient
    for j=1:length(J)
        Route{T(I(J(j)))}=[cRoute T(I(J(j)))];        
    end  
    % find minimum costed node which is not visited yet in this iteration
    [mn cr]=min(Cost(T));
    if isinf(mn)
        i=n;
    end
    curr=T(cr);
    % select selected route as forbitten route to not to select this nodes
    % again
    cRoute=Route{curr};
    Tabu=[Tabu curr];
end
```

### Dynamic Programming Algorithm

<a href="https://drive.google.com/uc?export=view&id=1FLM7elQ8Jc53lcG39A-xTr_rfhEKUFG1"><img src="https://drive.google.com/uc?export=view&id=1FLM7elQ8Jc53lcG39A-xTr_rfhEKUFG1" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>


```matlab
function [parent, route, cost]=dynamicpathplanning(Graph,Loc,Ind,startnode,endnode)
% calculate minimum costs path with recursive manner.

route=[];
cost=0;
dist=[startnode 0];
parent=[startnode nan];
n=size(Graph,1);
deep=0;
% call main recursive function, this function will call itself and give us
% route
[ds dist parent ] = sp_dp(Graph, endnode, dist,parent,Loc,Ind,deep); 

Route=[];
Route=[endnode ];
next=1;
while Route(end)~=startnode && ~isnan(next)
    curr=Route(end);
    xx=find(parent(:,1)==curr);
    next=parent(xx,2);
    if ~isnan(next)        
        cost=cost+costcal(Loc(Ind(curr),:),Loc(Ind(next),:));
        Route=[Route next];
    end
end
if Route(end)==startnode
    route=Route;    
else
    route=[];
    cost=inf;
end
```


## Visualization in Gazebo

<a href="https://drive.google.com/uc?export=view&id=1A4viL-m_JdXSKvhh3eN38lTsM6Y9yGFY"><img src="https://drive.google.com/uc?export=view&id=1A4viL-m_JdXSKvhh3eN38lTsM6Y9yGFY" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

```matlab
clear all
close all
clc
rosshutdown()
% number of nodes 
ns=100;
map=map_definition();
% generate random nodes
[map, nodelocation]= generate_node(map,ns);

% create undirected graph and its edges
[undirectedGraph,unedges]=generate_undirected_graph(map,nodelocation);

% create directed maps , its edges Index which shows which new node has the
% same location with original nodes, and biloc refer all new nods location
[bigraph,edges,nodIndex,biloc]=generate_directed_graph(undirectedGraph,nodelocation);


% define start and end point of simulation
startp=[5, 29, 0];
endp=[29, 20, 0];

% add start and end location as a new 2 nodes in undirected map.
% n+1 th node is start point, n+2th node is end point
[extungraph,exnodelocation,exunedges ]=addstartendpoint2ungraph(map,undirectedGraph,nodelocation,unedges,startp,endp);
exundnodIndex=[1:ns+2];
snodeund=ns+1;
enodeund=ns+2;
% update directed graph according to new added two nodes
[exbigraph,exbiloc,exedges,exnodIndex,startnode,endnode ]=addstartendpoint2bigraph(bigraph,extungraph,biloc,edges,nodIndex,exnodelocation,startp,endp);


% optimal path with astar on directional map
[Route] = astar(exbigraph,exbiloc,startnode,endnode);
astar_route=exbiloc(Route,:);
cost=pathcost(astar_route);
drawRoute('Astar',startnode,endnode,exnodelocation,exnodIndex,exunedges,Route,cost);
hold on;



% connect roscore
rosinit('127.0.0.1'); 
% create goal publisher
[pub,msg] = rospublisher('/move_base_simple/goal','geometry_msgs/PoseStamped');
msg.Header.FrameId='map';

% create transform reader to read robot location
tftree = rostf;
tftree.AvailableFrames;



pause(0.1);

% get robot position
pos=getrobotpose(tftree);

% set first node as a current route node 
curind=size(astar_route,1);




% if robot arrives last point in 0.2 error limit, then finish
while costcal2(astar_route(1,:),pos)>0.2
         
    % if robot arrives current point in 0.2 error limit, then select next
    while costcal2(astar_route(curind,:),pos)>0.2  
        
        % set target node position to robot's move_base node
        msg=mat2rospos(msg,astar_route(curind,:));
        % send message to ros
        send(pub,msg);             
        % get robots position
        pos=getrobotpose(tftree)
        % plot robot position on matlab figure
        plot(pos(1),pos(2),'ko');
        pause(0.1);        
        
    end 
    % go next node's position
    curind=curind-1;    
end
```

## PATH PLANNING WITH TWO ROBOTS MOVING SIMULTANEOUSLY

<a href="https://drive.google.com/uc?export=view&id=1Z_GfNSYsnnQFpHejlgkdohf4CYEtV3tY"><img src="https://drive.google.com/uc?export=view&id=1Z_GfNSYsnnQFpHejlgkdohf4CYEtV3tY" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

```matlab
% solving robot routing while there is another robot which locations along
% the time is known.

% define maximum time step size for two robot example
ts=20;

% take start and and point of another robot.
if scenario==0
    map_definition();
    hold on;
    for i=1:size(nodelocation,1)
        plot(nodelocation(:,1),nodelocation(:,2),'b*');
    end
    title('right click for other robot''s start and end points');
    [gx gy]=ginput(2);
    hold off;
end

% find the nearest node for given start and end point for othe robot.
dist=nodelocation-[gx(1) gy(1) 0];
dist=dist(:,1).^2+dist(:,2).^2;
[mn mni]=min(dist);
ostartnode=mni;
dist=nodelocation-[gx(2) gy(2) 0];
dist=dist(:,1).^2+dist(:,2).^2;
[mn mni]=min(dist);
oendnode=mni;

% find optimum path for another robot with djkstra on undirected map
% which we do not care about rotation for memory problem
[Route Cost] = dijkstra(extungraph,exnodelocation,ostartnode);
oroute=Route{oendnode};
% find optimum path for robot if there is not another robot.
[Route Cost] = dijkstra(extungraph,exnodelocation,snodeund);
org_route=Route{enodeund};


% create time-based graph.
[tgraph,tedge,tloc]= createtimegraph(extungraph,exunedges,exnodelocation,ts);
% remove the connection which are forbitten for the other robot.
[ctgraph,ctedge]= removeconnection(tgraph,tedge,oroute,tloc,ts);
% find best route for robot
[Route Cost] = dijkstra(ctgraph,tloc,snodeund);

% in solution set. the solution which was found in earlier is our solution
j=1;
solutions={};
scost=[];
for i=enodeund:size(extungraph,1):size(Route)
    if ~isempty(Route{i})
        rt=Route{i}-size(extungraph,1)*[0:length(Route{i})-1];        
        solutions{j}=rt;
        scost(j)=Cost(i);
        j=j+1;
    end
end

% simulate the robots path
simulate_simultaneous_move(ostartnode,oendnode,oroute,snodeund,enodeund,solutions{1},org_route,exnodelocation,exundnodIndex);


orglocs=exnodelocation(org_route,:);
ourlocs=exnodelocation(solutions{1},:);
costorg=pathcost(orglocs);
costour=pathcost(ourlocs);
title(['without algorithm cost:' num2str(costorg) ' with algorithm cost:' num2str(costour)]);
pause;
```

##  “Optimal multirobot path planning on graphs: complete algorithms and effective heuristics”

<a href="https://drive.google.com/uc?export=view&id=1T7Motb7njS5io-NQoE_iJlgPo2AbxUiD"><img src="https://drive.google.com/uc?export=view&id=1T7Motb7njS5io-NQoE_iJlgPo2AbxUiD" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

```matlab
% this demo is based on nxn grid graph as the paper shows
% robots are on the grid and they can move just 4 basic direction.
% this demo runs just for any number of robot.

ngrid=3;
nrobot=9;

% create bidirectional grid as the same example in the paper in fig1
[Graph Loc Edge]=GridGraph(ngrid);

% plot the created grid-graph
figure; plot(Loc(:,1),Loc(:,2),'bo');
axis([0 ngrid+1 0 ngrid+1]);
hold on;
for i=1:2:size(Edge,1)
    x1=Loc(Edge(i,1),1);
    x2=Loc(Edge(i,2),1);
    y1=Loc(Edge(i,1),2);
    y2=Loc(Edge(i,2),2);
    line([x1;x2],[y1;y2]);
end

% set the maximum time step
ts=7;

% r1 2nd cell, r2 4th cell and r3 6th cell as start position
%sp=[2 4 6 1 ];
sp=[9 5 6 8 3 1 2 4 7];
% r1 8nd cell, r2 6th cell and r3 4th cell as end position
%ep=[8 6 4 9 ] + ngrid*ngrid*(ts-1);
ep=[7 8 9 4 5 6 1 2 3] + ngrid*ngrid*(ts-1);

% create time based graph which has ts times more node in figure 5 in paper
[tG,tedge,tloc]= createtimegraph(Graph,Edge,Loc,ts);

% solve the problem with a*, it is not the same with paper since they used
% ILP solver.
[route] = astar3(tG,tloc,sp,ep,ts);

% show the reults on the screen
clr={'r','g','k','b','cy','y','r','g','b'};
shp={'ro','go','ko','bo','cyo','yo','r*','g*','b*'};
sz=[0.8,1.6,2,2.2,2.6,3, 3, 3,3];
i=1;
figure; hold on;
for j=1:nrobot
    plot(tloc(route(i,j),1),tloc(route(i,j),2),[clr{j} 'o'],'MarkerFaceColor',clr{j},'MarkerSize',10);
    pl(j,:)=[tloc(route(i,j),1) tloc(route(i,j),2)];
end
axis([0 ngrid+1 0 ngrid+1]);
for i=1:2:size(Edge,1)
    x1=Loc(Edge(i,1),1);
    x2=Loc(Edge(i,2),1);
    y1=Loc(Edge(i,1),2);
    y2=Loc(Edge(i,2),2);
    line([x1;x2],[y1;y2]);
end
hold off;

for i=2:size(route,1)
    for j=1:nrobot
        cl(j,:)=[tloc(route(i,j),1) tloc(route(i,j),2)];
    end
    
    for iter=0:30
        clf
        for i=1:2:size(Edge,1)
            x1=Loc(Edge(i,1),1);
            x2=Loc(Edge(i,2),1);
            y1=Loc(Edge(i,1),2);
            y2=Loc(Edge(i,2),2);
            line([x1;x2],[y1;y2]);
        end
        axis([0 ngrid+1 0 ngrid+1]);
        hold on;
        for j=1:nrobot
            plot(pl(j,1)+iter*(cl(j,1)-pl(j,1))/30,pl(j,2)+iter*(cl(j,2)-pl(j,2))/30,shp{j},'MarkerFaceColor',clr{j},'MarkerSize',10);
        end
        pause(0.1);
        hold off;
    end
    
    pl=cl;
end
```
