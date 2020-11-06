function [P1,P2] = longerLines(Point1,Point2,scale)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
A=makeLine(Point1,Point2,10);
slope=(A(2,:)-A(1,:))/(1/10);
P1=Point1;
P2=Point2+slope*(scale);

end

