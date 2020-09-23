function [x,y,z] = testgetxyz (d)
    r = 5;
    x = r*cos(d(1))*sin(d(2));
    y = r*sin(d(1))*sin(d(2));
    z = r*cos(d(2));
end