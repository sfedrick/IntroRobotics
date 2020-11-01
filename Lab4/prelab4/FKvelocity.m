function [v] = FKvelocity(qdot,qconfig)
J=CreateJac(qconfig,6);
v=J*qdot';
end

