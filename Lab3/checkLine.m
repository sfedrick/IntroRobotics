function [OkLine] = checkLine(P, node, obstacles)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here


%checks that each point on the line has an inverse kinematic solution if it
%doesn't the line is not okay
checkIKvalid(P, node, )

% create a discretized line in 3D space from the closest node in start tree --> point P
% and closest node in end tree to point P

%checks for collisions
checkCollision(line

OkLine=true;
end

