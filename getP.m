function [theta1, theta2, theta3, outOfPos] = getP (C, upperLim, lowerLim, constants)
theta1=[];
theta2=[];
theta3=[];
a1 = constants(1);
a2 = constants(2);
a3 = constants(3);
d5=constants(4);

outOfPos= -1;
%thetaup
[theta1,theta2,theta3]=elbowDown(C, constants);
[theta1,theta2,theta3]=elbowUp(C, constants);
%{
[theta1,theta2,theta3]=elbowUpTheta(C, constants,-1);
[theta1,theta2,theta3]=elbowUpTheta(C, constants,1);
[theta1,theta2,theta3]=elbowDownTheta(C, constants,-1);
[theta1,theta2,theta3]=elbowDownTheta(C, constants,1);
%}
if(C(1)==0 && C(2)==0)
    theta1=nan;
end

end 