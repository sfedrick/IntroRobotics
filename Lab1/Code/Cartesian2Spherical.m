function [r, theta, phi] = Cartesian2Spherical(x, y, z)
    r = sqrt(x^2 + y^2 + z^2);
    if (x == 0)
       theta = 0;
    else
       theta = atan(y/x);
    end
    if (y == 0)
        phi = 0;
    else
        phi = atan(sqrt(x^2 + y^2)/z);
    end
    
end