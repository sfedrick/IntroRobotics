% This function plots a straight line defined by a series of velocity
% vectors.
% Most of the function is pretty much the same as circlePlot so did not
% comment this

% initialize time linespace discretization
t = linspace(0,2*pi,50);
dt = t(2) - t(1);

% initialize radius
r = 50;

% initialize velocity values at each time step for circle

v = [1,1,1,0,0,0];
v=v*30;

v(1) = NaN;
v(4:6) = NaN;

hold on

jointDesired = 6;

q = [0,0,0,0,0]';
[jointPos,T] = calculateFK(q);
for joint=1:5
    linkPoint1 = jointPos(joint,:);
    linkPoint2 = jointPos(joint+1,:);    
    linePlot(linkPoint1,linkPoint2,1,[0,0,0],2);
end
for i=1:50
    qd = IKvelocity(v,q,jointDesired);
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

end

xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
hold off