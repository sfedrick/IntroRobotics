function [OkLine] = checkLine(P, node, obstacles)
% checkLine ensures that 

OkLine=true;
% create a discretized line in 3D space from the closest node in start tree --> point P
% and closest node in end tree to point P

% use FK to get start/end positions of all links


% modify each of the obstacles s.t. their dimensions are increased by check
% fk for each element of line 


smallRadius=5;
raddicheck=Inf;
step=100;


while(raddicheck>smallRadius)
    step=step+10;
    line=makeLine(node,P,step);
    L=length(line);
    [jointPositions,TN]=calculateFK(line(1,:));
    [jointPositions,TP]=calculateFK(line(2,:));
    Reali_1=TN(:,1:3);
    Reali=TP(:,1:3);
    raddicheck=norm(Reali_1-Reali);
end







for i=1:L
    [jointPositions,TN]=calculateFK(line(i,:));
    [row,col]=size(jointPositions);
    links=[];
    for j=1:row-1
        point1=jointPositions(j,:);
        point2=jointPositions(j+1,:);
        links=[links;point1,point2];  
    end
    starts=links(:,1:3);
    ends=links(:,4:6);
    [row,col]=size(obstacles);
    for k=1:row
        kcheck=detectCollision(starts, ends, obstacles(k,:));
        if(max(kcheck)==1)
            OkLine=false;
        end
    end
end



end

