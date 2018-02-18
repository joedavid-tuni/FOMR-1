function [tgraph,tedge,tloc]= createtimegraph(Graph,edge,loc,t)
% this fuction costruct time-based graph using normal graph.
nn=size(Graph,1);
% time based graoh has t times more nodes
tgraph=zeros(t*nn);
tedge=[];
tloc=loc;
eyeI=eye(nn);

for i=1:t-1
    tloc=[tloc;loc];
    % establish time based graph connections
    tgraph((i-1)*nn+1:i*nn,i*nn+1:(i+1)*nn)=Graph+eyeI;
    tmp1=edge(:,1)+(i-1)*nn;
    tmp2=edge(:,2)+(i)*nn;
    tedge=[tedge; tmp1 tmp2];
    tmp=[1:nn]';
    tmp3=tmp+(i-1)*nn;
    tmp4=tmp+(i)*nn;
    tedge=[tedge; tmp3 tmp4];
end