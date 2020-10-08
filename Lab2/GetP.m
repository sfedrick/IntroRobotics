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

  %elbow down
    %[theta1,theta2,theta3] = elbowDown(C, constants);
    [theta1,theta2,theta3] = elbowUp(C, constants);
   
    [theta1,solution]=theta1test(theta1);
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