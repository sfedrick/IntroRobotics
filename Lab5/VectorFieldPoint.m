function [PointVector] = VectorFieldPoint(pos,goal,obstacles,params)
%Calculates vector field at a point 
%   Detailed explanation goes here
%pos 1x3
%goal 1x3
%obstacle nx6 where n is the number of obstacles
%params are the parameters defined below 
AttStrength=params(1);
RepStrength=params(2);
Po=params(3);
dimension=length(pos);
%real field
% [numObs,trash]=size(obstacles);
% Repulse=[0,0,0]
% for i=1:numObs
%     obstacle=obstacles(i,:);
%     Ri=Repulsive(pos,obstacle,RepStrength,Po);
%     Repulse= Repulse+Ri;
% end 
% Attract=Attractive(pos,goal,AttStrength);
% PointVector=Repulse + Attract;

%test vector field 
PointVector=Repulsive(pos,obstacle,RepStrength,Po);

end

