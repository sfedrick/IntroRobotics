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

<<<<<<< HEAD:Lab0/Problem4.m
%Initialize DH variables
theta = 0; alpha = 0; a = 0; d = 0;
A = zeros(4);
=======
%DH Parameters
DH_params = [0, -pi/2, L1, theta1; 
            L2, 0, 0, theta2 + pi/2;
            L3, 0, 0, theta3 + pi/2;
            0, pi/2, 0, theta4 - pi/2;
            0, 0, L4+L5, theta5 + pi];
        
>>>>>>> refs/remotes/origin/master:Lab1/Code/Problem4.m
       
%Initialize T matrices, combined them into a single matrix to make easier
%to calculate
T = eye(4);

<<<<<<< HEAD:Lab0/Problem4.m
x = [];
y = [];
z = [];

for joint1 = -1.4:0.4:1.4
    for joint2 = -1.2:0.4:1.4
        for joint3 = -1.4:0.4:1.7
            for joint4 = -1.9:0.4:1.7
                for joint5 = -2:0.4:1.5
                    T = eye(4);
                    DH_params = [0, -pi/2, L1, joint1; 
                                L2, 0, 0, joint2 + pi/2;
                                L3, 0, 0, joint3 + pi/2;
                                0, pi/2, 0, joint4 - pi/2;
                                0, 0, L4+L5, joint5 + pi];
                            
                            %Calculate each intermediate homogeneous transformation matrix
                    for link = 1:5
                        a = DH_params(link,1);
                        alpha = DH_params(link,2);
                        d = DH_params(link,3);
                        theta = DH_params(link,4);

                        % calculate A
                        A = createA(a, alpha, d, theta);
                        disp('T'+link+' = ');
                        T = T*A; 
                    end
                    x = [x; T(1,4)];
                    y = [y; T(2,4)];
                    z = [z; T(3,4)];
                end
            end
        end
    end
=======
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
>>>>>>> refs/remotes/origin/master:Lab1/Code/Problem4.m
end

plot3(x,y,z,'.');
xlabel('X');
ylabel('Y');
zlabel('Z');

