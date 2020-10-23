function realPos = TestPoint(discPos, steps, dims)
%TESTPOINT spits out real position of a discretized point
% starts from the boundary's start point

% discPos (1 x 3): location in the discretized grid of a given test point (NOT real space dimensions)
% steps (1 x 1): # of steps to discretize by
% dims (dim x 2): size of the boundary and the start/ends of each boundary

realPos = [];

% calculate delta for each dimension
delta = [];
[row,col] = size(dims);
for r = 1:row
    delta(r) = abs(dims(r,1)-dims(r,2))/steps;
end

% calculates the real position of each location for each dimension
for i=1:length(discPos)    
    realPos(i) = discPos(i)*delta(i)+dims(i,1);
end

end

