function [ForceVector] = Attractive(pos,goal,AttStrength)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
ForceVector=-AttStrength*(pos-goal);
end

