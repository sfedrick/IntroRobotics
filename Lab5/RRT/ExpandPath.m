function [BigWay] = ExpandPath(Waypoints,steps)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
BigWay=[];
[row,col]=size(Waypoints);
    for i=1:row-1
        point1=Waypoints(i,:);
        point2=Waypoints(i+1,:);
        newline=makeLine(point1,point2,steps);
        BigWay=[BigWay;newline];
    end
end

