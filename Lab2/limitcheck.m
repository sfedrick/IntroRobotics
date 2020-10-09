function [output,solution] = limitcheck(joint,upper,lower)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
solution=0;
for i=1:length(joint)
    if(joint(i)<=upper && joint(i)>= lower)
        output=joint(i); 
        solution=1;
    end
end
end

