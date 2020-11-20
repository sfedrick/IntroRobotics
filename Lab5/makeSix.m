function [sixVec]=makeSix(Vec)
%turns any nx1 vector to an 6x1 vector this was useful for dealing with the
%different outputs of the jacobian when you varied the desired joint.
L=length(Vec);
    while(L<6)
        Vec(L+1)=0;
        L=L+1;
    end
 sixVec=Vec;
 [row,col]=size(sixVec);
 if(row~=1)
     sixVec=sixVec';
 end
end 