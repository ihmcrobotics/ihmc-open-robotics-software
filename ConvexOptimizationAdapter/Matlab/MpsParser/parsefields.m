%
% fields=parsefields(line)
%
% Returns a cell array of the fields in the input line, using spaces as
% delimiters.
%
function fields=parsefields(line)
fields=cell(0);
pt=1;
count=0;
while (pt <= length(line))
  if (line(pt)==' ') 
    pt=pt+1;
  else
    count=count+1;
    pt2=pt;
    while ((pt2 <= length(line)) && (line(pt2)~=' '))
      pt2=pt2+1;
    end
    pt2=pt2-1;
    fields{count}=line(pt:pt2);
    pt=pt2+1;
  end
end
