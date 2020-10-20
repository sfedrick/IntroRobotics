function [Nodes,Edges] = RRT(dims,Obstacles,q1,q2)
%RRT Summary of this function goes here
%   Detailed explanation goes here
% dims (dim x 2): size of the boundary and the start/ends of each boundary
NodeStart=[q1];
EdgeStart=[];
NodeEnd=[q2];
EdgeEnd=[];



found=false;
while(~found)
   P=randpoint(dims);
   Si=closeNode(P,NodeStart);
   Ei=closeNode(P,NodeEnd);
   S=NodeStart(Si);
   E=EdgeStart(Ei);
   LineS=makeLine(P,S);
   LineE=makeLine(P,E);
   Scheck1=checkLine(LineS);
   Echeck1=checkLine(LineE);
   
   
    
end

outputArg1 = inputArg1;
outputArg2 = inputArg2;
end

