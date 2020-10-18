function [outputArg1,outputArg2] = obstacle(xmin,ymin,zmin,x_max,y_max,z_max,color)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
start=[xmin,ymin,zmin];
endy=[x_max,y_max,z_max];
length=abs(start-endy);
plotcube(length,start,.8,color);
end

