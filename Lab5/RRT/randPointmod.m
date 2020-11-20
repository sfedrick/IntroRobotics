function [P] = randPointmod(percent,dims,nearQ)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[row,col]=size(nearQ);
modder=dims(:,2)-dims(:,1);

nearQ1=nearQ-percent*modder;
nearQ2=nearQ+percent*modder;
P=[];
    for i=1:col
        P(i)=rand(1)*(abs(nearQ1(i)-nearQ2(i)))+nearQ1(i);
    end
lowerlimcheck=P<dims(:,1);
upperlimcheck=P>dims(:,2);
for i=1:length(lowerlimcheck)
   if(lowerlimcheck(i)==1)
       P(i)=dims(i,1);
   elseif(upperlimcheck==1)
       P(i)=dims(i,2);
   end
end
P(6)=0;
end

