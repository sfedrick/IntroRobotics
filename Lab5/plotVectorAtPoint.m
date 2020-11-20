
function [] = plotVectorAtPoint(pos,goal,map,params, scale,window)
%PLOTVECTORATPOINT was used to validate the vector fields 

%Calculates vector field at a point 
%   Detailed explanation goes here
%pos 1x3
%goal 1x3
%obstacle nx6 where n is the number of obstacles
%params are the parameters defined below 

goalObst = [goal goal+10];

AttStrength=params(1);
RepStrength=params(2);
poScale = params(3);
attrRadius = params(4);

if (length(pos) <= 2)
    pos(3) = 0;
    goal(3) = 0;
end

%real field
map = loadmap(map);
[numObs,trash]=size(map.obstacles);
colors=[1,0,0;
        1,0,0;
        1,0,0;
        1,0,0;
        1,0,0];
Repulse=[0,0,0];
for i=1:numObs
    mapObstacle=map.obstacles(i,:);
    Ri=Repulsive(pos,mapObstacle,RepStrength,poScale);
    Repulse= Repulse+Ri;
end 
Attract=Attractive(pos,goal,AttStrength,attrRadius);
PointVector=Repulse + Attract;
disp('Magnitude of Resultant: '+string(norm(PointVector)));
disp('Vector:');
disp(PointVector);
disp('Magnitude of Attractive: '+string(norm(Attract)));
disp('Vector:');
disp(Attract);
disp('Magnitude of Repulsive: '+string(norm(Repulse)));
disp('Vector:');
disp(Repulse);
PointVector = PointVector*scale;
Attract = Attract*scale;
Repulse = Repulse*scale;

figure(2);
hold on
quiver3(pos(1),pos(2),pos(3),PointVector(1),PointVector(2),PointVector(3),'color',[0 0 1]);
quiver3(pos(1),pos(2),pos(3),Repulse(1),Repulse(2),Repulse(3),'color',[1 0 0]);
quiver3(pos(1),pos(2),pos(3),Attract(1),Attract(2),Attract(3),'color',[0 1 0]);

title('Scaling Factor: '+string(scale));
xlim([-window+pos(1) window+pos(1)]);
ylim([-window+pos(2) window+pos(2)]);
zlim([-window+pos(3) window+pos(3)]);
obstacle(map.obstacles,colors);
obstacle(goalObst,[0 1 0]);
xlabel('X');
ylabel('Y');
zlabel('Z');
%axis equal;
hold off



end

