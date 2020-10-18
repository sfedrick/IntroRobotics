nodeStart=[1,1,0];
nodeEnd=[1,2,0];
node2=[2,2,0];
hold on;
obstacle(-0.4,-0.4,0, 0.4,0.4,0,[1 0 0])
TestPlot(nodeStart, nodeStart, 0);
TestPlot(nodeEnd, nodeEnd, 0);
TestPlot(nodeStart, node2, 1);

xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
hold off;