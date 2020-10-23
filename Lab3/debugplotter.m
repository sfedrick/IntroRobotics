
start = [0,0,0,0,0,0];
goal =[0,1,1,1,1,1];



map = loadmap('map6.txt');
cube=map.obstacles;

hold on
thiccCube=expandObstacles(10,cube);
set(gca,'Fontsize',20);
colors=[1,1,0;
        0,1,1];

obstacle(cube,colors);
[path] = rrt(map, start, goal);
step=length(path);
colorgradient=makeLine([0,0,1],[1,0,0],step);
sizegradient=makeLine(0.75,5,step);



xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')

hold off
