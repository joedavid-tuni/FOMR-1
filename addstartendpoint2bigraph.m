function [exbigraph,exbiloc,exedges,exnodIndex,startnode,endnode ]=addstartendpoint2bigraph(bigraph,extungraph,biloc,edges,nodIndex,exnodelocation,startp,endp)
% this function add two newcoming nodes in directed graph.
% not need to construct again, just it adds nodes and its connections

nnode=size(extungraph,1)-2;

exbigraph=bigraph;
exbiloc=biloc;
exedges=edges;
exnodIndex=nodIndex;
szo=size(exbigraph,1);
szoo=szo;
sz=szo+1;
% find all conncetion from first coming node to another nodes on undirected
% graph
I=find(extungraph(nnode+1,:)==1);

exbiloc=[exbiloc;startp];
exnodIndex=[exnodIndex;nnode+1];
startnode=size(exnodIndex,1);

% for all connected nodes
for i=1:length(I)
    nn=I(i);
    % calculate angle
    ang=atan2( exnodelocation(nn,2)-startp(2),exnodelocation(nn,1)-startp(1));
    exbiloc=[exbiloc;startp(1) startp(2) ang];
    exnodIndex=[exnodIndex;nnode+1];
    % szo+1 is the first coming nodes order, and it needs to connect to its
    % nodes which has the same location but different angle. this nodes
    % support to turning movement
    exbigraph(szo+1,sz+1)=1;
    exedges=[exedges;szo+1 sz+1];
    
    % turning node needs to connect the orginal nodes on directed graph
    exbiloc=[exbiloc;exnodelocation(nn,1:2) ang];
    exnodIndex=[exnodIndex;nn];
    % sz+1 is first graph turned stuation, sz+2 the node which has the same
    % location with old connected node
    exbigraph(sz+1,sz+2)=1;
    exedges=[exedges;sz+1 sz+2]; 
    % also this node should connect the nodes which connect the node which
    % has same location in old nodes.
    J=find(exnodIndex==nn);
    % this operation remove the new nodes
    J(find(J>szoo))=[];
    % add finsding node's connection
    exbigraph(sz+2,J(2:2:end))=1;
    for j=2:2:length(J)
        exedges=[exedges;sz+2 J(j)];
    end
    sz=size(exbigraph,1);
end
% do the same things for second added node.

szo=size(exbigraph,1);
sz=szo+1;
I=find(extungraph(nnode+2,:)==1);
exbiloc=[exbiloc;endp];
exnodIndex=[exnodIndex;nnode+2];
endnode=size(exnodIndex,1);
for i=1:length(I)
    nn=I(i);
    ang=atan2( endp(2)-exnodelocation(nn,2),endp(2)-exnodelocation(nn,1));
    exbiloc=[exbiloc;endp(1) endp(2) ang];
    exnodIndex=[exnodIndex;nnode+2];
    exbigraph(sz+1,szo+1)=1;
    exedges=[exedges;sz+1 szo+1];
    
    exbiloc=[exbiloc;exnodelocation(nn,1:2) ang];
    exnodIndex=[exnodIndex;nn];
    exbigraph(sz+2,sz+1)=1;
    exedges=[exedges;sz+2 sz+1];
    
    J=find(exnodIndex==nn);
    J(find(J>szoo))=[];
    exbigraph(J(1:2:end),sz+2)=1;
    for j=1:2:length(J)
        exedges=[exedges;sz+2 J(j)];
    end
    sz=size(exbigraph,1);
end

[m n]=size(exbigraph);
if m>n
    exbigraph(:,n+1:m)=0;
elseif n<m
    exbigraph(n+1:m,:)=0;
end
    
