function[] = CheckStartGoal(map,qstart,qgoal)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
figure;
O=loadmap(map).obstacles;
bigRadius = 10;
color = [0 0 1;
            0 0 1;
            0 0 1;
            0 0 1;
            0 0 1;
            0 0 1;];
color = [1 0 0;
            1 0 0;
            1 0 0;
            1 0 0;
            1 0 0;
            1 0 0;];
cube= [10.001 -0.1 5 12.002 1 10;
         -10.001 -0.1 5 -12.002 1 10;];  
obstacle(O,abs(color-0.5))
obstacles=[O;cube];
obstacles = expandObstacles(bigRadius,obstacles);
 
 hold on;
     
 
 obstacle(obstacles,color)
 plotJointPos(qstart, [0,1,0],4)
 plotJointPos(qgoal, [1,0,0],4)
hold off;
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis equal
end

