J=[1,0,0;
    0,0,0;
    0,0,1]
v=[0,1,0]'
a=0;
Rj=rank(J);
Rv=rank([J,v]);
if(Rj==Rv)
    disp("this is feasible");
else
    disp("this is infeasible");
end