function [thiccBois] = expandObstacles(radius,obstacles)
%EXPANDOBSTACLES This function takes in an obstacle array with multiple
%obstacles and expands them based on an inputted radius
[row,col]=size(obstacles);
if(row~=0 && col ~=0)
    thiccBois = obstacles;
    thiccBois(:,1:3) = thiccBois(:,1:3)-radius;
    thiccBois(:,4:6) = thiccBois(:,4:6)+radius;
else
    thiccBois=[];
end

end

