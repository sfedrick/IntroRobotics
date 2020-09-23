%Problem 4
%function T = problem4(q)

discStep = 9;

%Link Lengths
L1 = 76.2;
L2 = 146.05;
L3 = 187.325;
L4 = 34;
L5 = 34;
       
%Initialize T matrices, combined them into a single matrix to make easier
%to calculate
T = eye(4);
x = [];
y = [];
z = [];

for joint1 = -1.4:2.8/discStep:1.4
    for joint2 = -1.2:2.6/discStep:1.4
        for joint3 = -1.4:3.1/discStep:1.7
            for joint4 = -1.9:3.6/discStep:1.7
                for joint5 = -2:3.5/discStep:1.5
                    T = eye(4);
                    DH_params = [0, -pi/2, L1, joint1; 
                                -L2, 0, 0, joint2 + pi/2;
                                -L3, 0, 0, joint3 + pi/2;
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
                        T = T*A; 
                    end
                    x = [x; T(1,4)];
                    y = [y; T(2,4)];
                    z = [z; T(3,4)];
                end
            end
        end
    end
end
plot3(x,y,z,'.','MarkerSize',1);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on
k = boundary(x,y,z,1);
axis equal
trisurf(k,x,y,z,'FaceColor','red','FaceAlpha',0.1);
title('Computed Workspace');
%end

