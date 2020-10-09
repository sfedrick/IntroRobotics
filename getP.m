function [theta1, theta2, theta3, outOfPos] = getP (C, upperLim, lowerLim, constants)
theta1=[];
theta2=[];
theta3=[];
a1 = constants(1);
a2 = constants(2);
a3 = constants(3);
d5=constants(4);

outOfPos1 = 0;
%thetaup
Ed = elbowDown(C, constants);
Eu=elbowUp(C, constants);
UpTu=elbowuptheta(C, constants,-1);
UpTd=elbowuptheta(C, constants,1);
DTu=elbowDownTheta(C, constants,-1);
DTd=elbowDownTheta(C, constants,1);

if(x0c==0 && y0c==0)
    theta1=nan;
end

end 