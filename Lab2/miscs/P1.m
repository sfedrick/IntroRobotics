


% Declare constants
d1 = 76.2;
a2 = 146.05;
a3 = 187.325;
d5 = 68; %mm

% Specify end effector rotation matrix
R0e = [0, 0, 1;
       0, -1, 0;
       1, 0, 0];
% R0e = [0, 1, 0;
%        0, 0, 1;
%        1, 0, 0];

% Specify the end effector position
x0e = a3+d5;
y0e = 0;
z0e = d1+a2;
P=[x0e,y0e,z0e];
[theta1,theta2,theta3,theta4,theta5]=GetO(R0e,P);

R3e_check = [cos(theta4-pi/2)*cos(theta5), -cos(theta4-pi/2)*sin(theta5), -sin(theta4-pi/2);
            sin(theta4-pi/2)*cos(theta5), -sin(theta4-pi/2)*sin(theta5), cos(theta4-pi/2);
            -sin(theta5), -cos(theta5), 0];

[jointPos_R0e, R0e_check] = calculateFK([theta1, theta2, theta3, theta4, theta5])