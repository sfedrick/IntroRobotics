initial=[0,0,0,0,0,0];
final=[6.5,6.5,6.5,6.5,6.5,6.5];
[jointPositions,T0e] = calculateFK(initial);



step=1000;
Q=makeLine(initial,final,step);

hold on;
[row, col] = size(jointPositions);
% Plot the robot links in the start configuration
for joint=1:row-1
    linkPoint1 = jointPositions(joint,:);
    linkPoint2 = jointPositions(joint+1,:);    
    linePlot(linkPoint1,linkPoint2,1,[0,1,0],1);
end
[row,col]=size(Q);
dq=Q(2,:)-Q(1,:);
dq(end)=[];
for i=1:row
    q=Q(i,:);
    q(end)=[];
    [pv,pq]=fkeval(dq,q,1/step);
    trajPlot(pv,false,'.b');
    %trajPlot(pq,false,'-.r*');
end
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis equal
hold off


