function [J] = CreateJac(q,pointinterest)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
[jointPositions,zaxis]=currentConfig(q);
Oe=jointPositions(pointinterest,:);
J=[];
    for i=1:pointinterest-1
      zi_1= zaxis(:,i);
      Si=createS(zi_1);
      Oi=Oe-jointPositions(i,:);
      Jvi=Si*Oi';
      Jwi=zi_1;
      Ji=[Jvi;Jwi];
      J=[J,Ji];
  
    end
end

