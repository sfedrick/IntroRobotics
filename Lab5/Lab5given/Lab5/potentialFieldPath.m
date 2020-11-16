function [path] = potentialFieldPath(map, qStart, qGoal)
%function [path] = potentialFieldPath(map, qStart, qGoal)
% This function plans a path through the map using a potential field
% planner
%
% INPUTS:
%   map      - the map object to plan in
%   qStart   - 1x6 vector of the starting configuration
%   qGoal:   - 1x6 vector of the goal configuration
%
% OUTPUTS:
%   path - Nx6 vector of the path from start to goal

M=loadmap(map);
obstacles=M.obstacles;
path=qStart;
isDone=false;
i=1;
    while (~isDone)
        [qNext, isDone]=potentialFieldStep(path(i,:), obstacles, qGoal);
        path=[path;qNext];
        i=i+1;
    end
end