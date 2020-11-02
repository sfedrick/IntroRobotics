function [v] = FKvelocity(qdot,qconfig,joint)
J=CreateJac(qconfig,joint);
v=J*qdot';
end

