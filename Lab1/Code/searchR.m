function [rval,thetaIndex, phiIndex]=searchR(rmatrix,theta,phi,anglestep)

anglephi=0:(2*pi)/anglestep:2*pi;
angletheta=0:(2*pi)/anglestep:2*pi;

if (phi < 0)
   phi = 360 + phi;
end
if (theta < 0)
   theta = 360 + theta;
end

phiIndex = floor(phi/anglestep);
thetaIndex = floor(theta/anglestep);

if (phiIndex == 0)
    phiIndex = 1;
end
if (thetaIndex == 0)
    thetaIndex = 1;
end

rval = rmatrix(thetaIndex,phiIndex);

end