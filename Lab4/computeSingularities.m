% Fill in this file with your code for Analysis question #4.

% # Discretizations for each joint variable
discStep = 25;

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

% Compute the end effector Cartesian coordinate for each permutation of 
% joint variable
joint=6;
hold on;
for joint1 = -1.4:2.8/discStep:1.4
    for joint2 = -1.2:2.6/discStep:1.4
        for joint3 = -1.4:3.1/discStep:1.7
            for joint4 = -1.9:3.6/discStep:1.7
                for joint5 = -2:3.5/discStep:1.5
                   qconfig=[joint1 ,joint2,joint3,joint4,joint5];
                   J=CreateJac(qconfig,joint);
                   J=round(J);
                   if(rank(J)<5)
                       print("percent")
                       
                        [jointPositions,T0e]=calculateFK(qconfig);
                        x=jointPositions(joint,:);
                        plot3(x(1),x(2),x(3),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF');
                   end 
                end
            end
        end
    end
end
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off;
