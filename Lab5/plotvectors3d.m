clc;
clear;
X=-5:2:5;
Y=-5:2:5;
Z=-5:2:5;
qval=[];

i=1;
for x=X
    for y=Y
        for z=Z
            forcei=VectorFieldPoint([x,y,z],[0,0,0],[0,1,2,3,4,5,6],[1,1,1]);
            qval(i,:)=[x,y,z,forcei(1),forcei(2),forcei(3)];
            i=i+1;
        end
    end
end
hold on
quiver3(qval(:,1),qval(:,2),qval(:,3),qval(:,4),qval(:,5),qval(:,6));
hold off