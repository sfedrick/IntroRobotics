function [localmin] = checkForLocalMin(qConfig,listQConfig,tol)
%Qconfig nx6 array containing the past qconfigs of supected local minimum
%qconfig nx6 current qconfig 
% 1x1 scalar representing the tolerance

%   Detailed explanation goes here
localmin=true;
[row,col]=size(listQConfig);
for i=1:row
    dq=listQConfig(i,:)-qConfig;
    dq=norm(dq);
    if(dq>tol)
        localmin=false;
    end
end

end

