clc;
clear;

start = [0,0,0,0,0,0];
goal = [0,0,1.1,0,0,0];
f1 = figure(1);
hold on;

% Plots the obstacle
cube = [130 -300 96.825  400 300 113.175;
    150 60  -50.0 400 66.7  350.0];

thiccCube=expandObstacles(10,cube);
colors=[1,0,0;
        0,1,0];
obstacle(thiccCube,colors);


lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)

dims = [lowerLim;upperLim]';

[Waypoints] = ItWerks(dims,cube,start,goal,3,1)

xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
hold off;

