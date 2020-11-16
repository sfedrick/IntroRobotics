clc;
clear;
X=-2:.2:4;
Y=-2:.2:4;
Z=-2:.2:4;
qval=[];
obst = [0,0,0,1,1,1];
goal = [2,2,2];
goalObst = [goal goal+.1];

i=1;
for x=X
    for y=Y
        for z=Z
            forcei=VectorFieldPoint([x,y,z],goal,obst,[5,1,1,3]);
            qval(i,:)=[x,y,z,forcei(1),forcei(2),forcei(3)];
            i=i+1;
        end
    end
end
hold on
quiver3(qval(:,1),qval(:,2),qval(:,3),qval(:,4),qval(:,5),qval(:,6),0);
obstacle(obst,[1 0 0]);
obstacle(goalObst,[0 1 0]);
hold off
xlabel('X');
ylabel('Y');
zlabel('Z');