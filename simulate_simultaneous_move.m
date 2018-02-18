function simulate_simultaneous_move(s1,e1,r1,s2,e2,r2,r3,exnodelocation,exnodIndex)
rl=max([length(r1) length(r2) length(r3)]);
r1(end+1:rl)=r1(end);
r2(end+1:rl)=r2(end);
r3(end+1:rl)=r3(end);

    
ns=size(exnodelocation,1);
map_definition();
hold on;

title('Black line is joint route planning algorithm, yellow line what robot moves without the algorithm');
xlabel('For clarity using pre-produced scenario, use scenario==1 in Line 90 of main.m file  ');

sn=exnodIndex(s1);
en=exnodIndex(e1);
sn2=exnodIndex(s2);
en2=exnodIndex(e2);
plot(exnodelocation(1:ns,1),exnodelocation(1:ns,2),'b*');
plot(exnodelocation(sn,1),exnodelocation(sn,2),'rx','markersize',10);
plot(exnodelocation(en,1),exnodelocation(en,2),'rx','markersize',10);
plot(exnodelocation(sn2,1),exnodelocation(sn2,2),'rx','markersize',10);
plot(exnodelocation(en2,1),exnodelocation(en2,2),'rx','markersize',10);

% for i=1:2:size(exunedges,1)    
%     x1=exnodelocation(exunedges(i,1),1);
%     x2=exnodelocation(exunedges(i,2),1);   
%     y1=exnodelocation(exunedges(i,1),2);
%     y2=exnodelocation(exunedges(i,2),2);  
%     line([x1;x2],[y1;y2],'linewidth',0.1);    
% end

for i=1:length(r1)-1
    x1=exnodelocation(exnodIndex(r1(i)),1);
    x2=exnodelocation(exnodIndex(r1(i+1)),1);   
    y1=exnodelocation(exnodIndex(r1(i)),2);
    y2=exnodelocation(exnodIndex(r1(i+1)),2);  
    line([x1;x2],[y1;y2],'color','r','linewidth',3);
    
    x1=exnodelocation(exnodIndex(r2(i)),1);
    x2=exnodelocation(exnodIndex(r2(i+1)),1);   
    y1=exnodelocation(exnodIndex(r2(i)),2);
    y2=exnodelocation(exnodIndex(r2(i+1)),2);  
    line([x1;x2],[y1;y2],'color','k','linewidth',2);  
    
    x1=exnodelocation(exnodIndex(r3(i)),1);
    x2=exnodelocation(exnodIndex(r3(i+1)),1);   
    y1=exnodelocation(exnodIndex(r3(i)),2);
    y2=exnodelocation(exnodIndex(r3(i+1)),2);  
    line([x1;x2],[y1;y2],'color','y','linewidth',1);
    pause();
    
end
hold off