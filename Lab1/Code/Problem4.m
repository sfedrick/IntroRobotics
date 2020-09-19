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
            L3, 0, 0, theta3 + pi/2;
            0, pi/2, 0, theta4 - pi/2;
            0, 0, L4+L5, theta5 + pi];
        
       
%Initialize T matrices, combined them into a single matrix to make easier
%to calculate
T = eye(4);

%Calculate each intermediate homogeneous transformation matrix
for link = 1:5
    a = DH_params(link,1);
    alpha = DH_params(link,2);
    d = DH_params(link,3);
    theta = DH_params(link,4);
    
    % calculate A
    A = createA(a, alpha, d, theta);
    disp('T'+link+' = ');
    disp(A);
    T = T*A; 
end
disp('T0e = ');
disp(T);

