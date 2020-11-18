function [qNext, isDone,force] = potentialFieldStep(qCurr, map, qGoal,tolerance,dt,params)
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

isDone=false;

[currentpos,currentposO]=calculateFK(qCurr);
[goalpos,goalposO]=calculateFK(qGoal);
dq=[0,0,0,0,0,0];
force = [];
for i=2:6
    df=VectorFieldPoint(currentpos(i,:),goalpos(i,:),map,params);
    df=df';
    force(i) = norm(df);
    Ji=calcJacobian(qCurr, i);
    Ji=Ji(1:3,:);
    dqi=Ji'*df;
    dqi=makeSix(dqi);
    dq=dqi+dq;
end

if(norm(qCurr-qGoal)<=tolerance)
    isDone=true;
end
dq=dq/norm(dq);
qNext=qCurr+dt*dq;

end
