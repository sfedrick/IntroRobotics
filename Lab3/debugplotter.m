
start = [1.5, 0.7,1.7,0.2,-0.91,15];
goal =[1,1.3,-1.5,1.5,0.0,0];



map = loadmap('map7.txt');
cube=map.obstacles;

hold on
thiccCube=expandObstacles(10,cube);
set(gca,'Fontsize',20);
colors=[1,1,0;
        0,1,1;
        0,.5,.5;
        .5,.5,0;
        1,.5,1];

obstacle(cube,colors);
[path] = rrt(map, start, goal);
step=length(path);
colorgradient=makeLine([0,0,1],[1,0,0],step);
sizegradient=makeLine(1,5,step);




xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis equal
hold off
