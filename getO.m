function[theta1,theta2,theta3,theta4,theta5,outOfPos]=GetO(R0e,P,upperLim,lowerLim)
    % Declare constants
    d1 = 76.2;
    a2 = 146.05;
    a3 = 187.325;
    d5 = 68; %mm
    
    % Calculate position of wrist center
    
    %truetheta1=atan2(P(2),P(1));
    r=[P(1),P(2),0]';
    z0=[0,0,1]';
    n=cross(z0,r);
    %we pull the desired x0e y0e and z0e axis from R0e
    x0e=R0e(1:3,1);
    y0e=R0e(1:3,2);
    z0e=R0e(1:3,3);
    zp=project(n,z0e);
    yp=project(n,y0e);
    xp=project(n,x0e);
    
    lossyp=abs(norm(y0e)-norm(yp));
    lossxp=abs(norm(x0e)-norm(xp));
    zf=zp/norm(zp);
    %unsure about this order 
    xf=cross(z0,zp);
    yf=cross(z0,zp);
    %find the normal vector to zp in the y3 x3 plane 
     % we determine the order of the cross product and therefore which vector (x0e or y0e) 
    %that  we project onto the the z0 r plane by how much length each vector
    % loses when we project it onto the y3 x3 plane 
    if(lossxp<lossyp)
        yf=cross(zf,xf);
    else
        xf=cross(yf,zf);
    end
    R0e=[xf yf zf];
    
    [x0c,y0c,z0c] = GetxC(R0e,P,d5);
    [theta1, theta2, theta3,outOfPos] = getP([x0c,y0c,z0c],upperLim, lowerLim,[d1,a2,a3,d5]);
    % Use FK to find R03
    [jointPos_R03, R03] = calculateFK_R03([theta1, theta2, theta3]);
 
    
   R3e=(R03')*R0e;
   theta4 = atan2(R3e(2,3),R3e(1,3));
   theta5 = atan2(-R3e(1,3),-R3e(2,3));
    
end