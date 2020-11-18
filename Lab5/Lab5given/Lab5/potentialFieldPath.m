function [path, forces] = potentialFieldPath(map, qStart, qGoal)
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
lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)
M=loadmap(map);
obstacles=M.obstacles;
path=qStart;
isDone=false;
i=1;
dt=0.01;
params=[5,10,150,30];
tolerance=0.01;

forces = [];
    while (~isDone)
        [qNext, isDone, force]=potentialFieldStep(path(i,:), obstacles, qGoal,tolerance,dt,params);
        forces = [forces;force];
        if (isnan(norm(qNext)))
            break;
        end
        path=[path;qNext];
        
        if (length(path) > 5000)
            break;
        end
        i=i+1;
    end
end