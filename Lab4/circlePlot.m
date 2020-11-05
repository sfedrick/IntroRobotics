
% initialize time linespace discretization
t = linspace(0,2*pi,50);
dt = t(2) - t(1);

% initialize radius
r = 1000;

% initialize velocity values at each time step for circle
v = zeros(6,length(t));
for i=1:length(t)
    v(2,i) = r*sin(t(i));
    v(3,i) = r*cos(t(i));
end

hold on

for i=1:length(v)
    qd = IKvelocity(v(1:6,i),[0,0,0,0,0,0],6);
%     q = qd*dt;
    %[jointPos,T] = calculateFK(qd);
%     for joint=1:5
%         linkPoint1 = jointPos(joint,:);
%         linkPoint2 = jointPos(joint+1,:);    
%         linePlot(linkPoint1,linkPoint2,1,[0,1,0],1);
%     end
    [pv,pq] = fkeval(qd,[0,0,0,0,0,0],dt)
    trajPlot(pv,false,'.k');
%     plot3(jointPos(3,1),jointPos(3,2),jointPos(3,3),'.b');
%     plot3(jointPos(4,1),jointPos(4,2),jointPos(4,3),'.r');
%     plot3(jointPos(5,1),jointPos(5,2),jointPos(5,3),'.b');
%     plot3(jointPos(6,1),jointPos(6,2),jointPos(6,3),'.k');
end
hold off
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;

% qd = IKvelocity(v(1:6,30),[0,0,0,0,0,0],6)
% [jointPos,T] = calculateFK(qd);
% disp(jointPos)