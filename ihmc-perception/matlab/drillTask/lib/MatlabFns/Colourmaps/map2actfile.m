% MAP2ACTFILE   Writes colourmap to .act file Adobe Colourmap Table
%
% Usage: map2actfile(map, fname)
%
% An Adobe Colourmap Table is a file of 256 sets of R G and B values written as
% bytes, a total of 768 bytes.
%

% PK June 2014

function map2actfile(map, fname)

    [N, chan] = size(map);
    if N ~= 256 | chan ~= 3
        error('Colourmap must be 256x3');
    end
    
    % Ensure file has a .act ending
    if ~strendswith(fname, '.act')     
        fname = [fname '.act'];
    end

    % Convert map to integers 0-255 and form into a single vector of
    % sequential RGB values
    map = round(map*255);
    map = map';
    map = map(:);

    fid = fopen(fname, 'w');    
    fwrite(fid, map, 'uint8');
    fclose(fid);

    
