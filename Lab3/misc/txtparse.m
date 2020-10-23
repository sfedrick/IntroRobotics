function [T] = txtparse(file,skip,regex,rsplit)
T={};
fid = fopen(file);
tline = fgetl(fid);
i=1;
while ischar(tline)
    tline = fgetl(fid);
    A{i}=tline;
    str =tline;
    expression = regex;
   
    try
        matchStr= regexp(str,expression,'match');
    catch
         matchStr={};
    end
    if(length(matchStr)>0)
       M=regexp(str,rsplit,'split');
       T{i}=M(skip:end);
       i=i+1;
    end
end

end

