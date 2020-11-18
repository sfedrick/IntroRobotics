function [] = plotPath(map, qStart, qGoal)
%PLOTPATH 

[path,forces] = potentialFieldPath(map, qStart, qGoal);


symbols=['o','.','*','+','s','d'];
colors=[1 1 0;
        0 1 0;
        0 0 1;
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
plotJointPos(qStart, [1 0 0],4);
[row,col] = size(path);
count = 1;
for i=1:row
    if (mod(count, floor(0.01*row)) == 0 || i<20)
        [jointPositions,T0i] = calculateFK(path(i,:));

        plotPos(jointPositions,symbols,colors);
    end
    count = count+1;
end
plotJointPos(qGoal, [0 1 0],4);
map = loadmap(map);

obstacle(map.obstacles,obstcolors);
axis equal
xlabel('X');
ylabel('Y');
zlabel('Z');

hold off

figure(2)
plot(1:length(forces),forces(:,5),'.k')

end

