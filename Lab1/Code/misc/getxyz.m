function [x,y,z]= getxyz(d)
 T = eye(4);
 L1 = 76.2;
L2 = 146.05;
L3 = 187.325;
L4 = 34;
L5 = 34;

 joint1=d(1);
 joint2=d(2);
 joint3=d(3);
 joint4=d(4);
 joint5=d(5);
 %need to fix DH parameters 
 DH_params = [0, -pi/2, L1, joint1; 
                    L2, 0, 0, joint2 + pi/2;
                    L3, 0, 0, joint3 + pi/2;
                    0, pi/2, 0, joint4 - pi/2;
                    0, 0, L4+L5, joint5 + pi];
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

x=T(1,4);
y=T(2,4);
z=T(3,4);

end 