function [Nodeindex] = closeNode(P,Nodes)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
least=inf;
%distance is based on L2 norm this can be changed to fit application
[row,col]=size(Nodes);
Nodeindex=[];
for i=1:row
    D=abs(Nodes(i)-P);
    newdistance=norm(D);
    if(newdistance<=least)
        least=newdistance;
        Nodeindex=i;
    end
end 
end

