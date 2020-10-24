%start = [1.3, 0.7,1.7,0.2,-0.91,15];
start = [0, 0,0.95,.9,0,0];
goal = start+0.01;
[jointPositionsStart,T0e] = calculateFK(start);
[jointPositionsEnd,T0e] = calculateFK(goal);

hold on;


% Plots the obstacle
% cube = [130 200 96.825  160 230 113.175;
%     50 100 150 70 130 180;
%     120 50 35 140 70 55;
%     120 75 100 135 95 120;
%     10 160 180 160 170 190];
cube= [10.001 -0.1 5 12.002 1 10;
        -10.001 -0.1 5 -12.002 1 10;];   
%%cube=loadmap('map1.txt').obstacles;

thiccCube=expandObstacles(10,cube);
colors=[1,1,0;
        0,1,1;
        0,.5,.5;
        .5,.5,0;
        1,.5,1];
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

