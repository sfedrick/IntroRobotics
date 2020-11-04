function [v] = FKvelocity(qdot,qconfig,joint)
J=CreateJac(qconfig,joint);
[row,col]=size(qdot);
    if(col==1)
        v=J*(qdot);
    else
        v=J*(qdot');
    end
end

