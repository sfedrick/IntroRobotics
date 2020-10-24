function [path] = rrt(map, start, goal)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%T=txtparse(map,2,'block ','\s+');

Obstacles=map.obstacles;
% for i=1:length(T)
%     row=[]
%     S = sprintf('%s ', T{i}{:});
%     D = sscanf(S, '%f');
%     Obstacles=[Obstacles;D'];
%  end

lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)
dims = [lowerLim;upperLim]';
%debug plotter
%smallpath=ItWerks(dims,Obstacles,start,goal,1,1);
%path=smallpath;

%rrt planner
cube = [10.001 -0.001 5 10.002 0 5;
        -10.001 -0.001 5 -10.002 0 5;];
Obstacles = [Obstacles;cube];

smallpath=ItWerks(dims,Obstacles,start,goal,0,0);
path=ExpandPath(smallpath,15);

end

