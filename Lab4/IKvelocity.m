function [qDesired] = IKvelocity(v,qconfig,joint)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

J=CreateJac(qconfig,joint);
a=0;
%removes NaN rows from J and V because they do not matter
for i=1:length(v)
    a=a+1;
    if(isnan(v(a)))
        v(a)=[];
        J(a,:)=[];
        a=a-1;
    end
end
%{
SquareJ=J'*J;
Jplus=SquareJ\J';

[row,col]=size(v);
if(col==1)
    qDesired=Jplus*v;
else
    qDesired=Jplus*v';
end
%}


[row,col]=size(v);
if(col==1)
    qDesired=J\v;
else
    v=v';
    qDesired=J\v;
end

Rj=rank(J);
Rv=rank([J,v]);
if(Rj==Rv)
    disp("this is feasible");
else
    disp("this is infeasible");
end

end

