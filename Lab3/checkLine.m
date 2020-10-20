function [OkLine] = checkLine(P, node, obstacles)
% checkLine ensures that 

OkLine=true;
% create a discretized line in 3D space from the closest node in start tree --> point P
% and closest node in end tree to point P

% use FK to get start/end positions of all links


% modify each of the obstacles s.t. their dimensions are increased by check
% fk for each element of line 

radius=5;
raddicheck=Inf;
step=100;
%{
while(raddicheck>radius)
    step=step+10;
    line=makeLine(node,P,step);
    L=length(line);
    [jointPositions,TN]=calculateFK(line(0));
    [jointPositions,TP]=calculateFK(line(1));
    Reali_1=TN(:,1:3);
    Reali=TP(:,1:3);
    raddicheck=norm(Reali_1-Reali);
end
%}

%delete when implemented corecttly 
line=makeLine(node,P,step);
L=length(line);
%delete the two above lines when implemented properly



%{
for i=1:L
    [jointPositions,TN]=calculateFK(line(i));
    [row,col]=size(jointPositions);
    lines=[];
    for j=1:row-1
        point1=jointPositions(j,:);
        point2=jointPositions(j+1,:);
        lines=[lines;point1,point2];  
    end
    starts=lines(:,1:3);
    ends=lines(:,3:6);
    [row,col]=size(obstacles);
    for k=1:row
        kcheck=detectCollision(starts, ends, obstacles(k,:));
        if(kcheck==true)
            OkLine=false;
        end
    end
    
end
%}
for i=2:L
    starts=line(i-1);
    ends=line(i);
 
    [row,col]=size(obstacles);
    for k=1:row
        kcheck=detectCollision(starts, ends, obstacles(k,:));
        if(kcheck==true)
            OkLine=false;
        end
    end
    
end



end

