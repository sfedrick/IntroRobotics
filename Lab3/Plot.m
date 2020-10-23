
start = [0,0,0,0,0.5,0];
goal = [1,1,1.1,0,0,0];



map = loadmap('Map1.txt');
cube=map.obstacles;
tic 
[path] = rrt(map, start, goal);
toc

[row,col]=size(path);
%[path] = astar(map, start, goal);
thiccCube=expandObstacles(10,cube);
colors=[1,1,0;
        0,1,1];
hold on
set(gca,'Fontsize',20);
obstacle(thiccCube,colors);
step=length(path);
colorgradient=makeLine([0,0,1],[1,0,0],step);
sizegradient=makeLine(0.75,5,step);
pause(5)
for i=1:row         
    %plot the two trees 
    plotJointPos(path(i,:),colorgradient(i,:),sizegradient(i));
    pause(0.1)
    shg
end
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
hold off
