function [J] = CreateJac(q,pointinterest)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
[jointPositions,zaxis]=currentConfig(q);
J=[];
if(pointinterest>0)
    Oe=jointPositions(pointinterest,:);
    for i=1:pointinterest-1
      zi_1= zaxis(:,i);
      Si=createS(zi_1);
      Oi=Oe-jointPositions(i,:);
      Jvi=Si*Oi';
      Jwi=zi_1;
      Ji=[Jvi;Jwi];
      J=[J,Ji];
  
    end
   if(pointinterest==1)
       Jvi=[0,0,0]';
       Jwi=[0,0,0]';
       J=[Jvi;Jwi];
   end
end
end

