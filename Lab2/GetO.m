function[theta1,theta2,theta3,theta4,theta5]=GetO(R0e,P,upperLim,lowerLim)
    % Declare constants
    d1 = 76.2;
    a2 = 146.05;
    a3 = 187.325;
    d5 = 68; %mm
    
    % Calculate position of wrist center
    [x0c,y0c,z0c] = GetxC(R0e,P,d5);
    
    [theta1, theta2, theta3] = GetP([x0c,y0c,z0c],upperLim, lowerLim,[d1,a2,a3,d5]);

    % Use FK to find R03
    [jointPos_R03, R03] = calculateFK_R03([theta1, theta2, theta3])
    inv_R03 = inv(R03);

    % Find R3e
    R3e = inv_R03*R0e;
    theta5 = acos(-R3e(3,2));
    theta4 = asin(-R3e(2,3));
    
end