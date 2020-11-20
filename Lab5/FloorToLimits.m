function [P] = FloorToLimits(P)
lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)

%forces the config to be within the joint limits 
lowerlimcheck=P<lowerLim;
upperlimcheck=P>upperLim;
for i=1:length(lowerlimcheck)
   if(lowerlimcheck(i)==1)
       P(i)=lowerLim(i);
   elseif(upperlimcheck==1)
       P(i)=upperlimcheck(i);
   end
end
end

