function [x,y,z] = Sphericalplot(R,angleStep)
% example of what x and y should look like
%x = -2:.2:2; 
% y = -2:.2:2;
% 
% [xx,yy] = meshgrid(x,y);
% zz = xx.*exp(-xx.^2-yy.^2);
x=[];
y=[];
z=[];
 anglephi=0:(2*pi)/angleStep:2*pi;
 angletheta=0:(2*pi)/angleStep:2*pi;
 rshape=size(R);
row= rshape(1);
col=rshape(2);
for i=1:row
    for j=1:col
        r=R(i,j);
        theta=angletheta(i);
        phi=anglephi(j);
        xnew=r*cos(theta)*sin(phi);
        ynew=r*sin(theta)*sin(phi);
        znew=r*cos(phi);
        x=[x;xnew];
        y=[y;ynew];
        z=[z;znew];
         x=[x;xnew];
        y=[y;ynew];
        z=[z;-znew];
    end
end 

end 