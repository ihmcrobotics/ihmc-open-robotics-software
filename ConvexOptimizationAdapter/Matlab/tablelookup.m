%
% value=tablelookup(table,key)
%
function value=tablelookup(table,key)
%
% Hash the key.  
%
h=hash(key,table.n);
%
%
%
while ((strcmp(table.keys{h},key)==0) & (length(table.keys{h})~=0))
  h=mod(h,table.n);
  h=h+1;
end
%
% Return the hit or Nan for failed lookup.
%
if (strcmp(table.keys{h},key)==1)
  value=table.values{h};
else
  value=NaN;
end
