function [ctgraph,ctedge]= removeconnection(tgraph,tedge,route,tloc,t)
% this function removes the connection on time-based graph if the other
% robot is in that node. and also its connections
ctedge=tedge;
ctgraph=tgraph;
nn=size(tgraph,1)/t;

route(end+1:t)=route(end); %% other robot stay at the last node till the end of simulation

for i=1:length(route)
    node=route(i)+(i-1)*nn;    
    % remove the node's connection which has the same location with other
    % robot
    rloc=tloc(node,1:2);
    tmp=tloc( (i-1)*nn+1:i*nn,1:2)-rloc;
    dst=(tmp(:,1).^2+tmp(:,2).^2);
    I=find(dst<0.0001);
    ctgraph((i-1)*nn + I,:)=0;
    for j=1:length(I)
        J=find(ctedge(:,1)==(i-1)*nn + I(j));
        ctedge(J,:)=[];
    end 
    
    
end
% remove the edges which has intersection between other robots route

for i=1:length(route)-1
    n1=route(i)+(i-1)*nn;
    n2=route(i+1)+(i)*nn;    
    p1x=tloc(n1,1);
    p1y=tloc(n1,2);
    p2x=tloc(n2,1);
    p2y=tloc(n2,2);
    I=[];
    II=find( ctedge(:,1) > (i-1)*nn & ctedge(:,1) <= i*nn);
    for j=1:length(II)      
        e1x=tloc(ctedge(II(j),1),1);
        e1y=tloc(ctedge(II(j),1),2);
        e2x=tloc(ctedge(II(j),2),1);
        e2y=tloc(ctedge(II(j),2),2);
        % check if two connection has any intersection or not
        [xi,yi] = polyxpoly([e1x e2x],[e1y e2y],[p1x p2x],[p1y p2y]);
        if ~isempty(xi)
            I=[I;II(j)];
            ctgraph(ctedge(II(j),1),ctedge(II(j),2))=0;
        end
      
    end
    ctedge(I,:)=[];
end
        
        
    
    
    
    
    
    

    

