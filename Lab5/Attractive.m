function [ForceVector] = Attractive(pos,goal,AttStrength,radius)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

dist = norm(pos-goal);
if (dist >= radius)
    ForceVector = -AttStrength*(pos-goal)/dist;
else
    ForceVector=-AttStrength*(pos-goal);
end


end

