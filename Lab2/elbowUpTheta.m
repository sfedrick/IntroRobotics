function [theta1,theta2,theta3] = elbowUpTheta(C, constants,updown)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    x0c = C(1);
    y0c = C(2);
    z0c = C(3);
    
    d1 = constants(1);
    a1 = d1;
    a2 = constants(2);
    a3 = constants(3);
    theta1 = atan2(y0c,x0c)+pi*updown;
    r1 = sqrt(y0c^2 + x0c^2);
    r2 = z0c - d1;
    phi2 = atan2(r1,r2);
    r3 = sqrt(r1^2 + r2^2);
    phi1 = acos((a3^2-a2^2-r3^2)/(-2*a2*r3));
    theta2 = phi2-phi1+pi*updown;
    phi3 = acos((r3^2-a2^2-a3^2)/(-2*a2*a3));
    theta3 = pi/2-phi3+pi*updown;
end

