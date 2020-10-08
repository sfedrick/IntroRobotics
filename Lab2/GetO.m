function[theta1,theta2,theta3,theta4,theta5,outOfPos]=GetO(R0e,P,upperLim,lowerLim)
    % Declare constants
    d1 = 76.2;
    a2 = 146.05;
    a3 = 187.325;
    d5 = 68; %mm
    
    % Calculate position of wrist center
    [x0c,y0c,z0c] = GetxC(R0e,P,d5);
    
    [theta1, theta2, theta3,outOfPos] = GetP([x0c,y0c,z0c],upperLim, lowerLim,[d1,a2,a3,d5]);

    % Use FK to find R03
    [jointPos_R03, R03] = calculateFK_R03([theta1, theta2, theta3]);
    inv_R03 = inv(R03);

    % Find R3e
    R3e = R03'*R0e;
    theta5 = asin(-R3e(3,1));
    theta4 = asin(-R3e(2,3));
    
    theta1 = real(theta1);
    theta2 = real(theta2);
    theta3 = real(theta3);
    theta4 = real(theta4);
    theta5 = real(theta5);
    
end