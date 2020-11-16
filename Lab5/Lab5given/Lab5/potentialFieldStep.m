function [qNext, isDone] = potentialFieldStep(qCurr, map, qGoal)
% function [qNext, isDone] = potentialFieldStep(qCurr, map, qGoal)
% This function exectures one step of the potential field planner.
%
% INPUTS:
%   qCurr:   - 1x6 vector of the robots current configuration
%   map:     - the map object to plan in
%   qGoal:   - 1x6 vector of the goal configuration
%
% OUTPUTS:
%   qNext - 1x6 vector of the robots next configuration
%   isDone - a boolean that is true when the robot has reached the goal or
%            is stuck. false otherwise
tolerance=0.001;
isDone=false;
dt=0.01;
params=[5,1,1,3];
[currentpos,currentposO]=calculateFK(qCurr);
[goalpos,goalposO]=calculateFK(qGoal);
dq=[0,0,0,0,0,0];
for i=2:6
    df=VectorFieldPoint(currentpos(i,:),goalpos(i,:),map,params);
    df=df';
    Ji=calcJacobian(qCurr, i);
    Ji=Ji(1:3,:);
    dqi=Ji'*df;
    dqi=makeSix(dqi);
    dq=dqi+dq;
end

if(norm(qCurr-qGoal)<=tolerance)
    isDone=true;
end

qNext=qCurr+dt*dq;
end
