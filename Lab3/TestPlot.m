function [] = TestPlot(nodeOld, nodeNew, hasLine)
% Plots line between 2 nodes and also plots the 2nd node

plot(nodeNew);
if (hasLine == true)
    line = norm(nodeOld-nodeNew);
    step = line/1000;
    %plot(nodeOld,nodeNew,'k.'); 
    x = 1:step:line;
    plotLine = createPlotLine(nodeOld, x, step);
    plot3(plotLine(:,1), plotLine(:,2), plotLine(:,3),'-');
end

end

function plotLine = createPlotLine(nodeOld, x, step)
    plotLine = [nodeOld];
    for disc=2 : length(x)
        plotLine = [plotLine; disc*step+plotLine(disc-1,:)];
    end
end
