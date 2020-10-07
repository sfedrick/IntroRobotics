function [theta1, theta2, theta3, outOfPos] = GetP (C, upperLim, lowerLim, constants)
     
    x0c = C(1);
    y0c = C(2);
    z0c = C(3);
    
    a1 = constants(1);
    a2 = constants(2);
    a3 = constants(3);
    
    outOfPos1 = 0;
    outOfPos = zeros(3,1);

    % Calculate joint rotations for theta1, theta2, theta3
%{
    l2 = a2 + a1;
    r = sqrt(x0c^2 + y0c^2);
    theta1 = atan2( y0c,x0c);
    s = sqrt(r^2+z0c^2);
    theta3 = -pi/2 + acos((s^2-l2^2-a3^2)/(-2*l2*a3));
    theta3=-theta3;
    omega = pi/2 - theta3; 
    alpha = atan2((l2+a3*cos(omega)),(a3*sin(omega)));
    beta = atan2(r,z0c);
    theta2 = pi/2 - alpha - beta;
    %}
  %elbow down

    theta1 = atan2( y0c,x0c);
    [theta1,outofPos(1)]=theta1test(theta1);
    z0c=z0c-a1;
    theta3=-pi/2 -acos((x0c^2+y0c^2+z0c^2-a2^2-a3^2)/(2*a2*a3));
    theta2=pi/2 -atan2(z0c,sqrt(x0c^2+y0c^2))+atan2(a3*sin(-pi/2-theta3),(a2+a3*cos(-pi/2-theta3)));
    %}
      % Check that theta1,2,3 are valid and inside joint limits
      
      
      theta=[theta1,theta2,theta3];
    if (isnan(theta2) || isnan(theta3))
            outOfPos1 = -1;
    end 
    
    if((theta2 < lowerLim(2) || theta2 > upperLim(2)) && outOfPos1~=-1)
        %theta2 = -theta2;
    end 
    
    if ((theta3 < lowerLim(3) || theta3 > upperLim(3)) && outOfPos1~=-1)
        %theta3 = -pi/2 + alpha-beta;
    end
    
    for i = 1:3
        if (isnan(theta(i)))
            outOfPos(i) = -1;
        end
        if (outOfPos(i) ~= -1)
            if (theta(i) < lowerLim(i) || theta(i) > upperLim(i))
                outOfPos(i) = 1;
            end
        end
    end
end