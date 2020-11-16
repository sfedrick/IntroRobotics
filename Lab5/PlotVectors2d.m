clc;
clear;
X=-2:.2:4;
Y=-2:.2:4;
qval=[];
obst = [0,0,0,1,1,1;
        -1, -1, -1, -0.9, -0.9, -0.9];
colors = [1 0 0; 0 0 1];
goal = [2,2,2];
goalObst = [goal goal+.1];


i=1;
for x=X
    for y=Y
        forcei=VectorFieldPoint([x,y],goal,obst,[1,10,2.5,3]);
        qval(i,:)=[x,y,forcei(1),forcei(2)];
        i=i+1;
    end
end
hold on
quiver(qval(:,1),qval(:,2),qval(:,3),qval(:,4));
obstacle(obst,colors);
obstacle(goalObst,[0 1 0]);
hold off