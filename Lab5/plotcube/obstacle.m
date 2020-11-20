function [outputArg1,outputArg2] = obstacle(obstacles,color)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
[row,col]=size(obstacles);
    for i=1:row
        start=obstacles(i,1:3);
        endy=obstacles(i,4:6);
        length=abs(start-endy);
        plotcube(length,start,.7,color(i,:));
    end

end

