function [rval,thetaIndex, phiIndex]=searchR(rmatrix,theta,phi,anglestep)

anglephi=0:(2*pi)/anglestep:2*pi;
angletheta=0:(2*pi)/anglestep:2*pi;
binsize=(2*pi)/anglestep;
if (phi < 0)
   phi = 2*pi + phi;
elseif(phi>2*pi)
   phi=mod(phi,(2*pi));
end
if (theta < 0)
   theta = 2*pi + theta;
elseif(phi>2*pi)
   theta=mod(theta,(2*pi));
end

phiIndex = floor(phi/binsize);
thetaIndex = floor(theta/binsize);

if (phiIndex == 0)
    phiIndex = 1;
end
if (thetaIndex == 0)
    thetaIndex = 1;
end

rval = rmatrix(thetaIndex,phiIndex);

end