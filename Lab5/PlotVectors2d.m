clc;
clear;
X=-10:1:10;
Y=-10:1:10;
qval=[];

i=1;
for x=X
    for y=Y
        forcei=VectorFieldPoint([x,y],[0,0],[0,1,2,3,4,5,6],[1,1,1]);
        qval(i,:)=[x,y,forcei(1),forcei(2)];
        i=i+1;
    end
end
hold on
quiver(qval(:,1),qval(:,2),qval(:,3),qval(:,4));
hold off