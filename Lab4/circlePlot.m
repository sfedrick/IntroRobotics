
% initialize time linespace discretization
t = linspace(0,2*pi,50);
dt = t(2) - t(1);

% initialize radius
r = 50;

% initialize velocity values at each time step for circle
v = zeros(6,length(t));
for i=1:length(t)
    v(2,i) = r*sin(t(i));
    v(3,i) = r*cos(t(i));
end
v(6,:)=-1;
%v(1,:) = NaN;
%v(4:6,:) = NaN;

hold on

jointDesired = 6;

q = [0,0,0,0,0]';
[jointPos,T] = calculateFK(q);
for joint=1:5
    linkPoint1 = jointPos(joint,:);
    linkPoint2 = jointPos(joint+1,:);    
    linePlot(linkPoint1,linkPoint2,1,[0,0,0],2);
end
for i=1:length(v)
    qd = IKvelocity(v(1:6,i),q,jointDesired);
    for j=1:length(q)
        if (j >= jointDesired)
            qd = [qd; q(j)]; 
        end
    end
    
    q = q+qd*dt;
    fakeQ = [q; 0];
    [jointPos,T] = calculateFK(fakeQ);
   
    [pv,pq] = fkeval(qd,q,dt);
    trajPlot(T,true,'.k');
    trajPlot(pv,false,'.k');
    if(mod(i,10)==0)
        display(T)
    end
%     plot3(jointPos(3,1),jointPos(3,2),jointPos(3,3),'.b');
%     plot3(jointPos(4,1),jointPos(4,2),jointPos(4,3),'.r');
%     plot3(jointPos(5,1),jointPos(5,2),jointPos(5,3),'.b');
%     plot3(jointPos(6,1),jointPos(6,2),jointPos(6,3),'.k');

end

xlabel('X');
ylabel('Y');
zlabel('Z');
axis([-260 260 -260 260 0 260]);
axis equal;
hold off
% qd = IKvelocity(v(1:6,30),[0,0,0,0,0,0],6)
% [jointPos,T] = calculateFK(qd);
% disp(jointPos)