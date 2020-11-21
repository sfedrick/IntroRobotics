function [path, forces] = potentialFieldPath(map, qStart, qGoal)
%function [path] = potentialFieldPath(map, qStart, qGoal)
% This function plans a path through the map using a potential field
% planner
%
% INPUTS:
%   map      - the map object to plan in
%   qStart   - 1x6 vector of the starting configuration
%   qGoal:   - 1x6 vector of the goal configuration
%
% OUTPUTS:
%   path - Nx6 vector of the path from start to goal
lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)
%load in map and define parameters
if(min(class(map)=='string'))
    M=loadmap(map);
else
    M=map;
end
obstacles=M.obstacles;
bigRadius = 10;
%obstacle representing self collision with the base 
 cube= [10.001 -0.1 5 12.002 1 10;
         -10.001 -0.1 5 -12.002 1 10;];  
obstacles=[obstacles;cube];
%expands the obstacles to take in to account self collision
obstacles = expandObstacles(bigRadius,obstacles);
%sets the openness of the end effector to zero and records the final
%desired value to output at the end of the path
extension=qGoal(6);
qStart(6)=0;
qGoal(6)=0;
path=qStart;
isDone=false;
i=1;
dt=0.01;
%defines params 
% Attrative Strength=params(1);
% Repulsive Strength=params(2);
% range of repulsion poScale = params(3);
% range of quadratic repulsion attrRadius = params(4);
params=[1,10,50,30];
tolerance=0.1;
skip=20;
localMinAttemps=1;
listQConfig=zeros(skip,6);
atlocalmin=false;
badq=false;
slowWalking=0;
IveHadEnough=1000;
forces = [];
    while (~isDone)
        [qNext, isDone, force]=potentialFieldStep(path(i,:), obstacles, qGoal,tolerance,dt,params);
        qNext=FloorToLimits(qNext);
        forces = [forces;force];
        if (isnan(norm(qNext)))
            display("robot in obstacle try decreasing time step or changing start position")
            badq=true;
        end
        if(badq)
           qNext=path(end,:);
           badq=false;
           dt=dt*0.90;
           params(2)=params(2)+10;
           i=i-1;
        else
            path=[path;qNext];
        end
        n=mod(i,skip)+1;
        listQConfig(n,:)=qNext;
        if(mod(i,100)==0)
            atlocalmin=checkForLocalMin(qNext,listQConfig,0.1);
        end
        
        if(atlocalmin)
            %use rrt to get the next qnext
            localMinAttemps=localMinAttemps+1;
            minPath=rrt(map, qNext, qGoal);
            if (isnan(minPath))
                display("rrt broke me");
            end
            display("number of random rrt steps used");
            display(localMinAttemps)
            for m=2:localMinAttemps
                qNext=minPath(m,:);
                qNext(6)=0;
                path=[path;qNext];
            end
            atlocalmin=false;
        end
        if(norm(path(end)-path(end-1))<0.001 && dt<0.1)
            slowWalking=slowWalking+1;
            if(slowWalking>IveHadEnough)
                dt=dt*1.10;
                params(1)=params(1)+10;
                slowWalking=0;
            end
        end
        if(mod(i,IveHadEnough)==0 && params(1)<2*params(2)&& dt>=0.1)
            params(1)=params(1)+10;
            
        end

       if(i>5000)
           display("I broke")
           break;
       end
       i=i+1;
    end
    path(end,6)=extension;
   
end
  