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
