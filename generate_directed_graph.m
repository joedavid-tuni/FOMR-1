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