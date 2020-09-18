%Problem 4

%Link Lengths
L1 = 76.2;
L2 = 146.05;
L3 = 187.325;
L4 = 34;
L5 = 34;

%Joint Variable Angles
theta1 = -pi/2;
theta2 = 0;
theta3 = pi/4;
theta4 = 0;
theta5 = pi/2;

%DH Parameters
DH_params = [0, -pi/2, L1, theta1; 
            L2, 0, 0, theta2 + pi/2;
            L3, 0, 0, theta3 + pi/2
            0, pi/2, 0, theta4 - pi/2;
            0, 0, L4+L5, theta5 + pi];
        
%Initialize DH variables
theta = 0; alpha = 0; a = 0; d = 0;
A = zeros(4);
       
%Initialize T matrices, combined them into a single matrix to make easier
%to calculate
T = zeros(20,4);

%Calculate each intermediate homogeneous transformation matrix
for link = 1:size(DH_params, 1)
    a = DH_params(link,1);
    alpha = DH_params(link,2);
    d = DH_params(link,3);
    theta = DH_params(link,4);
    
    %Calculate DH Transformation Matrix A
    A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
        sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
        0, sin(alpha), cos(alpha), d;
        0, 0, 0, 1];
    
    %Replace section in T with A
    T(4*link-3:4*link,1:4) = A;
end;

%Pull out T's and set them to their own variables
T01 = T(1:4, 1:4);
T12 = T(5:8, 1:4);
T23 = T(9:12, 1:4);
T34 = T(13:16, 1:4);
T45 = T(17:20, 1:4);
    
%Calculate overall homogeneous matrix
T_final = T01*T12*T23*T34*T45;

disp(T_final);
