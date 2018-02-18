function [subgraph, Conn, subedge]=generate_subgraph(nodecon)
% this function generate subgraph of every nodes in undirected graph

subedge=[];
conn=find(nodecon==1);
sm=length(conn);

% every subgraph has 2 times all connection to this node
subgraph=zeros(2*sm,2*sm);
for i=1:2:2*sm
    for j=2:2:2*sm
        % every odd number node connect to even number node
        subgraph(i,j)=1;
        subedge=[subedge; i j];
    end
end
for i=2:2:2*sm
    % every even number node connect to its incoming node
   subgraph(i,i-1)=1;
   subedge=[subedge; i i-1]; 
end
Conn=zeros(1,2*sm);
Conn(1:2:end)=conn;
Conn(2:2:end)=conn;