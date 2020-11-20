function [ForceVector] = Attractive(pos,goal,AttStrength,radius)
%this is a function defining the attractive field 

dist = norm(pos-goal);
if (dist >= radius)
    ForceVector = -AttStrength*(pos-goal)/dist;
else
    ForceVector=-AttStrength*(pos-goal);
end


end

