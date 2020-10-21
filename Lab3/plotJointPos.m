function [] = plotJointPos(P, color)
%PLOTJOINTPOS 

% Plot the robot links in the start configuration
[jointPositions,T0e]=calculateFK(P);

[row, col] = size(jointPositions);
for joint=1:row-1
    linkPoint1 = jointPositions(joint,:);
    linkPoint2 = jointPositions(joint+1,:);    
    TestPlot(linkPoint1,linkPoint2,1,color);
end
end

