function [time, totalPathReal, totalPathConfig] = calculatePathDistance(isRRT, map)

    tic;
    start = [0,0,0,0,0,0];
    goal = [0,0,1.4,0,0,0];

    lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
    upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)

    map = loadmap(map);
    
    dims = [lowerLim;upperLim]';
    if (isRRT)
        [path] = rrt(map, start, goal);
    else
        [smallpath] = astar(map, start, goal);
        [path] = expandPath(smallpath, 100);
    end

    % Calculate waypoint path of end effector in real distance and config space distance
    [row,col] = size(path);
    jointPos = [];
    totalPathConfig = 0;
    for i=1:row-1

        for j=1:6
            % compute config space norm
            totalPathConfig = totalPathConfig + norm(path(i+1,j)-path(i,j));
        end
        [jointPositions,T0e] = calculateFK(path(i,:));
        %get the end effector position
        jointPos = [jointPos; jointPositions];
    end

%     steps = 10;
%     [row, col] = size(jointPos);
%     jointPosition = [];
%     for i=1:row-1
%         %newLine = makeLine(jointPos(i,:),jointPos(i+1,:),steps);
%         jointPosition = [jointPosition; newLine];
%     end


    totalPathReal = 0;
    [row, col] = size(jointPos);
    for i=1:row/6-1
        for j=1:6
            % Calculate path distance from current waypoint to next
            totalPathReal = totalPathReal + norm(jointPos(j+i*6,:)-jointPos(j+(i-1)*6,:));
        end
    end
    time = toc;
end