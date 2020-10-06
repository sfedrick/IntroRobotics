function [theta1, theta2, theta3, outOfPos] = GetP (C, upperLim, lowerLim, constants)
     
    x0c = C(1);
    y0c = C(2);
    z0c = C(3);
    
    a1 = constants(1);
    a2 = constants(2);
    a3 = constants(3);
    d5 = constants(4);
    
    outOfPos1 = 0;
    outOfPos = zeros(3,1);

    % Calculate joint rotations for theta1, theta2, theta3
    l2 = a2 + a1;
    r = sqrt(x0c^2 + y0c^2);
    theta1 = atan2(y0c, x0c);
    s = sqrt(r^2+z^2);
    theta3 = -pi/2 + acos((s^2-l2^2-a3^2)/(-2*l2*a3));
    omega = pi/2 - theta3;
    alpha = atan2((a3*sin(omega)),(l2+a3*cos(omega)));
    beta = atan2(z0c,r);
    theta2 = pi/2 - alpha - beta;
    
    % Check if any of the joint limits are out of position
    theta = [theta1, theta2, theta3];
    
      % Check that theta1,2,3 are valid and inside joint limits
    if (isnan(theta2) || isnan(theta3))
            outOfPos1 = -1;
    end 
    
    if((theta2 < lowerLim(2) || theta2 > upperLim(2)) && outOfPos1~=-1)
        theta2 = -theta2;
    end 
    
    if ((theta3 < lowerLim(3) || theta3 > upperLim(3)) && outOfPos1~=-1)
        theta3 = -pi/2 + alpha-beta;
    end
    
    for i = 1:3
        if (isnan(theta(i)))
            outOfPos(i) = -1;
        end
        if (outOfPos(i) ~= -1)
            if (theta(i) < lowerLim(i) || theta(i) > upperLim(i))
                outOfPos = 1;
            end
        end
    end
end