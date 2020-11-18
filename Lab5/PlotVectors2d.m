clc;
clear;

% AttStrength=params(1);
% RepStrength=params(2);
% poScale = params(3);
% attrRadius = params(4);
params=[1,1,300,5];
qval=[];
map = loadmap("map1.txt");
goal=[10,50,100];
obst = map.obstacles;
B = map.boundary;
X=B(1):50:B(4);
Y=B(2):50:B(5);
z= 96;
goalObst = [goal goal+10];
colors=[1,0,0;
        1,0,0;
        1,0,0;
        1,0,0;
        1,0,0];

i=1;
for x=X
    for y=Y
        forcei=VectorFieldPoint([x,y,z],goal,obst,params);
        qval(i,:)=[x,y,forcei(1),forcei(2)];
        i=i+1;
    end
end
hold on
quiver(qval(:,1),qval(:,2),qval(:,3),qval(:,4));
obstacle(obst,colors);
obstacle(goalObst,[0 1 0]);
hold off