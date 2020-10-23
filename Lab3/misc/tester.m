start = [0,0,0,0,0,0];
goal = start+0.001;
[jointPositionsStart,T0e] = calculateFK(start);
[jointPositionsEnd,T0e] = calculateFK(goal);

hold on;


% Plots the obstacle
cube = [130 -300 96.825  400 300 113.175;
    150 60  -50.0 400 66.7  350.0];
cube=loadmap('map1.txt').obstacles;

thiccCube=expandObstacles(10,cube);
colors=[1,0,0;
        0,1,0];
obstacle(thiccCube,colors);

success = 0;
checkLineBool = checkLine(start,goal,cube);
if (checkLineBool)
    success = 1;
end



% Plot the robot links in the start configuration
[row, col] = size(jointPositionsEnd);
for joint=1:row-1
    linkPoint1 = jointPositionsEnd(joint,:);
    linkPoint2 = jointPositionsEnd(joint+1,:);    
    TestPlot(linkPoint1,linkPoint2,success,'g',1);
end

% Plots the nodes regardless if there is no collision or not
for joint=1:row
    linkPoint1 = jointPositionsEnd(joint,:); 
    TestPlot(linkPoint1,linkPoint1,0,[0 0 1],1);
end

xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis equal
hold off;

