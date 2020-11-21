function [isCollided] = detectConfigCollision(map,qconfig)
%detects if a single configuration is in collision with any obstacles
if(min(class(map)=='string')||min(class(map)=='char'))
    loadedmap = loadmap(map);
else
    loadedmap =map;
end

obstacles = loadedmap.obstacles;
 cube= [10.001 -0.1 5 12.002 1 10;
         -10.001 -0.1 5 -12.002 1 10;]; 
obstacles=[obstacles;cube];

bigRadius = 10;
thiccObstacles = expandObstacles(bigRadius,obstacles);
 [jointPositions,T0i] = calculateFK(qconfig);
 [numobst,trash]=size(thiccObstacles);
  isCollided=false;
    for k=1:numobst
        for m=1:5
            isCollided = detectCollision(jointPositions(m,:),jointPositions(m+1,:),thiccObstacles(k,:));
            if(isCollided)
                break;
            end
        end
         if(isCollided)
                break;
         end
    end
end

