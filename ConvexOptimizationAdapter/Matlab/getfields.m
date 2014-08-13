%
% [line,fields]=getfields(fid)
%
% Reads the next nonempty, non-comment line of the input file and
% returns the fields of this line in a cell array.
%
function [line,fields]=getfields(fid)
%
% Read a line and parse it.
%
line=fgetl(fid);
if (line == -1)
  error('Unexpected end of file.');
end
line=stripcomments(line);
fields=parsefields(line);
%
% Skip through blank and comment lines until we get something real.
%
while ((length(line)==0) | (length(fields)==0))
  line=fgetl(fid);
  if (line == -1)
    error('Unexpected end of file.');
  end
  line=stripcomments(line);
  fields=parsefields(line);
end

