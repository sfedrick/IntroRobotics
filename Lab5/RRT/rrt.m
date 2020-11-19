function [path] = rrt(map, start, goal)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%T=txtparse(map,2,'block ','\s+');
map=loadmap(map);
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
bigRadius = 10;
thiccObstacles = expandObstacles(bigRadius,Obstacles);
%debug plotter
%smallpath=ItWerks(dims,Obstacles,start,goal,1,1);
%path=smallpath;

%rrt planner conservative self collisions much slower \
%{

cube =[10.001 -10 5 15.002 10 10;
        -15.001 -10 5 -10.002 10 10;];
    %}
% rrt planner less conservative
% cube= [10.001 -0.1 5 12.002 1 10;
%         -10.001 -0.1 5 -12.002 1 10;];   
% Obstacles = [thiccObstacles;cube];

smallpath=ItWerks(dims,thiccObstacles,start,goal,0,0);
path=ExpandPath(smallpath,20);

end

