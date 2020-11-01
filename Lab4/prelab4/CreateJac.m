function [J] = CreateJac(q,pointinterest)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
[jointPositions,zaxis]=calculateFKvelocity(q);
Oe=jointPositions(pointinterest,:);
J=[];
    for i=1:pointinterest-1
      zi_1= zaxis(:,i);
      Si=createS(zi_1);
      Oi=Oe-jointPositions(i,:);
      Jvi=Si*Oi';
      J=[J,Jvi];
      
    end
end

