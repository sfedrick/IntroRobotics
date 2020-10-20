function [OkLine] = checkLine(P, node, obstacles)
% checkLine ensures that 

% create a discretized line in 3D space from the closest node in start tree --> point P
% and closest node in end tree to point P

% use FK to get start/end positions of all links

% modify each of the obstacles s.t. their dimensions are increased by
% 2*radius of current joint

% check for collisions
checkCollision(line

OkLine=true;
end

