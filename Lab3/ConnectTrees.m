function [Waypoints] = ConnectTrees(EdgeStart,EdgeEnd,NodeStart,NodeEnd)
%UNTITLED Summary of this function goes here
Waypoints=[];
%   Walk through the edgestart matrix 
StartFinish=false;
[row,col]=size(EdgeStart);
search=EdgeStart(row,:);
%Cindex is the row of the matrix we are searching through
Cindex=[];
while(~StartFinish)
   %current index gives the next row because edge is a 
   %symetric matrix so Eij=Eji
   Cindex=find(search==-1);
   search=EdgeStart(Cindex,:);
   if(Cindex==1)
       StartFinish=true;
   end
    Waypoints=[NodeStart(Cindex,:);Waypoints];
end 



EndFinish=false;
[row,col]=size(EdgeEnd);
Cindex=row;
Waypoints=[Waypoints;NodeEnd(Cindex,:)];

search=EdgeEnd(row,:);
while(~EndFinish)
   %current index gives the next row because edge is a 
   %symetric matrix so Eij=Eji
   Cindex=find(search==-1);
   search=EdgeEnd(Cindex,:);
   if(Cindex==1)
       EndFinish=true;
   end
    Waypoints=[Waypoints;NodeEnd(Cindex,:)];
end 

end

