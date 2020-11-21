% generate random configurations

lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; 

randomConfigs = zeros(10,6);

for i=1:length(randomConfigs)
    for j=1:6
        xmin=lowerLim(j);
        xmax=upperLim(j);
        randNum=xmin+rand(1)*abs(xmax-xmin);
        randomConfigs(i,j) = randNum;
    end
end

count = 0;

[row, col] = size(randomConfigs);
for i=1:row
    [path, forces] = potentialFieldPath("map1.txt", [0 0 0 0 0 0], randomConfigs(i,:));
    if (path ~= 5000)
        count = count + 1
    end
end