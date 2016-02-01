% MAP2ERMAPPERLUTFILE Writes ER Mapper LUT colour map file
%
% Usage: map2ermapperlutfile(map, filename, name, description)
%
% Arguments:    map - N x 3 rgb colour map of values 0-1.
%          filename - Filename of LUT file to create.  A .lut ending is added
%                     if the filename has no suffix.
%              name - Optional string to name the colour map. This is used
%                     by ER Mapper to construct the lookup table menu list.
%       description - Optional string to describe the colour map.
%
% The ER Mapper LUT file format is also used ny MapInfo
%
% See also: READERMAPPERLUTFILE, READIMAGEJLUTFILE

% 2014 Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au

% PK September 2014

function map2ermapperlutfile(map, filename, name, description)
    
    if ~exist('name','var'), name = basename(namenpath(filename)); end
    if ~exist('description','var'), description = ''; end
    
    [N, chan] = size(map);
    if chan ~= 3
        error('Colourmap must be Nx3');
    end
    
    % Ensure file has a .lut ending
    if ~strendswith(filename, '.lut')     
        filename = [filename '.lut'];
    end
    
    [fid, msg] = fopen(filename, 'wt');
    error(msg);

    % Scale colour map values to 16 bit ints
    map = round(map*(2^16 - 1));
    
    fprintf(fid, 'LookUpTable Begin\n');
    fprintf(fid, '    Name        = "%s"\n', name);
    fprintf(fid, '    Description = "%s"\n', description);
    fprintf(fid, '    NrEntries   = %d\n', N);
    fprintf(fid, '    LUT = {\n', N);
    
    for n = 1:N
        fprintf(fid,   '%8d %8d %8d %8d\n',n-1, map(n,1), map(n,2), map(n,3));
    end
    
    fprintf(fid, '    }\n');
    fprintf(fid, 'LookUpTable End\n');
    
    fclose(fid);
