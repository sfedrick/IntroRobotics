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
 

    % Find R3e
    %find the desired rotation in terms of the wrist center orientation
    R3e = R03'*R0e;
    %we have R3e written relative to the wrist center rotation frame 
    % so we can write the normal vector as the basis z3 vector in the wrist
    % center frame
    n=[0,0,1]';
%     z0 = [0;0;1];
%     R30 = R03';
%     n = R30(1:3,3);
    %we pull the desired x3d y3d and z3d axis in R3e
    x3d=R3e(1:3,1);
    y3d=R3e(1:3,2);
    z3d=R3e(1:3,3);
    %we project x3d y3d and z3d axis onto the y3 x3 plane
    zp=project(n,z3d);
    yp=project(n,y3d);
    xp=project(n,x3d);
    % we determine the order of the cross product and therefore which vector (x3d or y3d) 
    %that  we project onto the the y3 x3 plane by how much length each vector
    % losses when we project it onto the y3 x3 plane 
    lossyp=abs(norm(y3d)-norm(yp));
    lossxp=abs(norm(x3d)-norm(xp));
    zf=zp/norm(zp);
    %find the normal vector to zp in the y3 x3 plane 
    normalzf=[zf(2),zf(1),0];
    normalzf=normalzf/norm(zf);
    if(lossxp<lossyp)
        xf=normalzf;
        yf=cross(zf,xf);
    else
        yf=normalzf;
        xf=cross(yf,zf);
    end
    
   R3e=[xf;yf;zf']*R3e;
    theta5 = asin(-R3e(3,1));
    theta4 = asin(-R3e(2,3));
    
    
   %{ 
    theta1 = real(theta1);
    theta2 = real(theta2);
    theta3 = real(theta3);
    theta4 = real(theta4);
    theta5 = real(theta5);
    %}
end