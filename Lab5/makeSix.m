function [sixVec]=makeSix(Vec)
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