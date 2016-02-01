%
% newtable=addtotable(table,key,value)
%
function table=addtotable(table,key,value)
%
% First check to see if we need to expand the table.
%
if ((table.count > 0.9*table.n) | (table.count > table.n-2))
%
% Error for now.
%
  error('Too many entries in table.');
else
%
% Add to an existing table.
%
  h=hash(key,table.n);
  while ((strcmp(table.keys{h},key)==0) && (length(table.keys{h})~=0))
    h=mod(h,table.n);
    h=h+1;
  end
  if (strcmp(table.keys{h},key)==1)
%
% Already in the table, just return.
%
    table.values{h}=value
    return
  else
    table.count=table.count+1;
    table.keys{h}=key;
    table.values{h}=value;
    return
  end
end
