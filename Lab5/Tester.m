clc;
clear;
% AttStrength=params(1);
% RepStrength=params(2);
% poScale = params(3);
% attrRadius = params(4);
params=[1,10,100,1];
goal=[50,100,20];
pos=[ 250     -200   93];
stringmap="map1.txt";

AttStrength=params(1);
RepStrength=params(2);
poScale = params(3);
attrRadius = params(4);

plotvectors3d(stringmap,params, goal)
scale=20;
window=100;
plotVectorAtPoint(pos,goal,stringmap,params, scale,window)
M=loadmap(stringmap);
[origin,pr]=circleObstacle(M.obstacles);
Repulsive(pos,M.obstacles,RepStrength,poScale);