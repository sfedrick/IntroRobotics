function [path,forces] = plotPath(map, qStart, qGoal)
%PLOTPATH this takes any map qstart and qgoal and plots their output 

[path,forces] = potentialFieldPath(map, qStart, qGoal);


symbols=['o','.','*','+','s','d'];
colors=[1 1 0;
        0 1 0;
        0.5 0.5 1;
        1 0 1;
        0 1 1;
        1 0.5 0]; 
obstcolors = [0 0 1;
            0 0 1;
            0 0 1;
            0 0 1;
            0 0 1;
            0 0 1;];
figure(1);
hold on

[row,col] = size(path);
count = 1;
for i=1:row
    if (mod(count, floor(0.01*row)) == 0 || i>row-100||i<100)
        [jointPositions,T0i] = calculateFK(path(i,:));

        plotPos(jointPositions,symbols,colors);
    end
    count = count+1;
end


map=loadmap(map);
obstacles=map.obstacles;
bigRadius = 10;
 cube= [10.001 -0.1 5 12.002 1 10;
         -10.001 -0.1 5 -12.002 1 10;];  
obstacles=[obstacles;cube];
obstacles = expandObstacles(bigRadius,obstacles);



obstacle(obstacles,obstcolors);
axis equal
xlabel('X');
ylabel('Y');
zlabel('Z');
plotJointPos(qStart, [1 0 0],4);
plotJointPos(qGoal, [0 1 0],4);
hold off

figure(2)
plot(1:length(forces),forces(:,5),'.k')

end

