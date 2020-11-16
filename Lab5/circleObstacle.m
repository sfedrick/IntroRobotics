function [pr, origin] = circleObstacle(obstacle)
% Calculates po, pr, and origin of rect obstacle. Basically turns the obstacle
% into a spherical obstacle.
% If po is smaller than 0.01, then establish it as a lower limit

dx = abs(obstacle(4) - obstacle(1));
dy = abs(obstacle(5) - obstacle(2));
dz = abs(obstacle(6) - obstacle(3));

originx = dx/2+obstacle(1);
originy = dy/2+obstacle(2);
originz = dz/2+obstacle(3);

origin = [originx, originy, originz];

% calculate cross edges
ce1 = norm([obstacle(1),obstacle(2),obstacle(3)]-[obstacle(4),obstacle(5),obstacle(6)]);
ce2 = norm([obstacle(1),obstacle(5),obstacle(6)]-[obstacle(4),obstacle(2),obstacle(3)]);
ce3 = norm([obstacle(1),obstacle(5),obstacle(3)]-[obstacle(4),obstacle(2),obstacle(6)]);
ce4 = norm([obstacle(1),obstacle(2),obstacle(6)]-[obstacle(4),obstacle(5),obstacle(3)]);

pr = max([ce1,ce2,ce3,ce4])/2;

end