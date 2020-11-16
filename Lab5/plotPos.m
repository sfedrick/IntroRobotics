function [ForceVector] = plotPos(pos,symbols,colors)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%colors is a n by 3 array of rgb colors
%symbols is an nx1 array of symbols 
%position is an n
[row,col]=size(pos);
    for i=1:row
        posi=pos(i,:);
        plot3(posi(1),posi(2),pos(3),symbols(i),'color',colors(i,:),'markersize',10)
    end 

end