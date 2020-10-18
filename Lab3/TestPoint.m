function realPos = TestPoint(discPos, delta)
%TESTPOINT spits out position of a discetized point in real space

realPos = [];
for i=1:discPos    
    realPos(i) = discPos(i)*delta;
end

end

