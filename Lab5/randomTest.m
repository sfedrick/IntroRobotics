% generate random configurations

lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; 

randomConfigs = [];
lengthRand = 0;
desiredNum = 0;
map = "map1.txt";
loadedmap = loadmap(map);
obstacles = loadedmap.obstacles;
 cube= [10.001 -0.1 5 12.002 1 10;
         -10.001 -0.1 5 -12.002 1 10;]; 
obstacles=[obstacles;cube];

bigRadius = 10;
thiccObstacles = expandObstacles(bigRadius,obstacles);
[numobst,trash] = size(thiccObstacles);

while(lengthRand < desiredNum)
    newq=[0 0 0 0 0 0];
    for j=1:6
        xmin=lowerLim(j);
        xmax=upperLim(j);
        newq(j)=xmin+rand(1)*abs(xmax-xmin);
    end
    [jointPositions,T0i] = calculateFK(newq);
    
    for (k=1:numobst)
        for (m=1:5)
            isCollided = detectCollision(jointPositions(m,:),jointPositions(m+1,:),thiccObstacles(k,:));
        end
    end
    if (~isCollided)
        randomConfigs = [randomConfigs; newq];
    end
    lengthRand = lengthRand + 1;
end

newq(end,:) = [1 0 0 0 0 0];
lengthRand = lengthRand + 1;

count = zeros(lengthRand,1);
rrtbreak = zeros(lengthRand,1);

for i=1:lengthRand
    [path, forces] = potentialFieldPath("map1.txt", [0 0 0 0 0 0], randomConfigs(i,:));
    if (length(path) >= 5000 || isnan(path(end,6)))
        if (isnan(path(end,6)))
            rrtbreak(i) = 1;
        end
    else
        count(i) = 1;
    end
end

disp('Randomly generated configurations');
disp(randomConfigs);
disp('Number of desired tests');
disp(desiredNum);
disp('Number of successes');
disp(count);
disp('Number of rrt breaks');
disp(rrtbreak);