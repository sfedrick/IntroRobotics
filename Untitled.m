[jp,T]=calculateFK([0,0,0,0,0,0]);
color='.k';
R = T(1:3,1:3);
    d = T(1:3,4)';

    % Construct x, y, z lines starting at each point
    x = R(1:3,1)';
    y = R(1:3,2)';
    z = R(1:3,3)';
plot3(d(1),d(2),d(3),color,'MarkerSize',10); 


[P1,P2]=longerLines(d,x+d,10);
linePlot(P1,P2,1,[1,0,0],2); % x = red
linePlot(d,(y+d),1,[0,1,0],2); % y = green
linePlot(d,(z+d),1,[0,0,1],2); % z = blue
axis equal;