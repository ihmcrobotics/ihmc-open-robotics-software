%
% table=newtable(n)
%
function table=newtable(n)
%
% Default to 1000 entries.
%
if (nargin == 1)
  table.n=n;
else
  table.n=1000;
end
%
%  Set up initial empty cell arrays.
%
table.keys=cell(table.n,1);
table.values=cell(table.n,1);
%
% Initialize to a count of 0.
%
table.count=0;
