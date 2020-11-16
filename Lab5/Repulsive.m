function [ForceVector] = Repulsive(pos,obstacle,RepStrength,poScale)
% calculates repulsive force 

% translate rect obstacle into spherical
[pr, obstOrigin] = circleObstacle(obstacle);

po = poScale*pr;

lowerlim = 0.1;
if (po < lowerlim)
    po = lowerlim;
end

pi = norm(pos - obstOrigin);

if (po > pi)
    dp = pos-obstOrigin;
    dp = dp/norm(dp); % normalize
    ForceVector = RepStrength*((1/pi)-(1/po))*(1/(pi^2))*dp;
else
    ForceVector = [0, 0, 0];
end
end
