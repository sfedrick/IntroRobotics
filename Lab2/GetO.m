function[theta1,theta2,theta3,theta4,theta5]=GetO(R0e,P)
    % Declare constants
    d1 = 76.2;
    a2 = 146.05;
    a3 = 187.325;
    d5 = 68; %mm

    % Calculate position of wrist center
    [x0c,y0c,z0c] = GetxC(R0e,P,d5);
    % Calculate joint rotations for theta1, theta2, theta3
    theta1 = atan2(y0c,x0c);
    r1 = sqrt(y0c^2 + x0c^2);
    r2 = z0c - d1;
    phi2 = atan2(r1,r2);
    r3 = sqrt(r1^2 + r2^2);
    phi1 = acos((a3^2-a2^2-r3^2)/(-2*a2*r3));
    theta2 = phi2-phi1
    phi3 = acos((r3^2-a2^2-a3^2)/(-2*a2*a3));
    theta3 = pi/2-phi3

    % Use FK to find R03
    [jointPos_R03, R03] = calculateFK_R03([theta1, theta2, theta3])
    inv_R03 = inv(R03);

    % Find R3e
    R3e = inv_R03*R0e;
    theta5 = acos(-R3e(3,2));
    theta4 = asin(-R3e(2,3));
    
end