%
% lineout=stripcomments(linein)
%
function lineout=stripcomments(linein)
if linein(1)=='*'
  lineout='';
else
  lineout=linein;
end
