function [thiccBois] = expandObstacles(radius,obstacles)
%EXPANDOBSTACLES This function takes in an obstacle array with multiple
%obstacles and expands them based on an inputted radius

thiccBois = obstacles;
thiccBois(:,1:3) = thiccBois(:,1:3)-radius;
thiccBois(:,4:6) = thiccBois(:,4:6)+radius;

end

