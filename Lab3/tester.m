start = [.4,0,0,0,0,0];
goal = start+0.001;
[jointPositionsStart,T0e] = calculateFK(start);
[jointPositionsEnd,T0e] = calculateFK(goal);

hold on;


% Plots the obstacle
obstacle(130,-300, 96.825,  400, 300, 113.175,[1 0 0]);
obstacle(150, 60,  -50.0, 400, 66.7,  350.0,[0 1 0]); 


cube = [130 -300 96.825  400 300 113.175;
    150 60  -50.0 400 66.7  350.0];


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
    TestPlot(linkPoint1,linkPoint2,success,'g');
end

% Plots the nodes regardless if there is no collision or not
for joint=1:row
    linkPoint1 = jointPositionsEnd(joint,:); 
    TestPlot(linkPoint1,linkPoint1,0,'b');
end

xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
hold off;

