function [] = plotvectors3d(map, goal)



qval=[];
map = loadmap(map);
obst = map.obstacles;
B = map.boundary;
X=B(1):50:B(4);
Y=B(2):50:B(5);
Z = B(3):50:B(6);
goalObst = [goal goal+10];

i=1;
for x=X
    for y=Y
        for z=Z
            % AttStrength=params(1);
            % RepStrength=params(2);
            % poScale = params(3);
            % attrRadius = params(4);
            forcei=VectorFieldPoint([x,y,z],goal,obst,[1,3,1,3]);
            qval(i,:)=[x,y,z,forcei(1),forcei(2),forcei(3)];
            i=i+1;
        end
    end
end
figure(1);
hold on
quiver3(qval(:,1),qval(:,2),qval(:,3),qval(:,4),qval(:,5),qval(:,6));
obstacle(obst,[1 0 0]);
obstacle(goalObst,[0 1 0]);
hold off
xlabel('X');
ylabel('Y');
zlabel('Z');
figure(3);
obstacle(obst,[1 0 0]);
end