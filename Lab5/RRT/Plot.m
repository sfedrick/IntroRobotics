
start = [0,0,0,0,0,0];
goal = [-1,0,1,0,0,0];



map = loadmap('map4.txt');
cube=map.obstacles;
tic 
[path] = rrt('map4.txt', start, goal);

toc

[row,col]=size(path);
thiccCube=expandObstacles(10,cube);
colors=[1,1,0;
        0,1,1;
        0,.5,.5;
        .5,.5,0;
        1,.5,1];
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
axis equal
hold off
