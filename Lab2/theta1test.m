function [output,solution]=theta1test(value)
output=value;
if(-1.4<value<1.4)
    output=value;
end

if(value>0)
  output=output-pi;
elseif(value<0)
  output=output+pi;
end


if(-1.4<output<1.4)
    solution=1;
else
    solution=-1;
end