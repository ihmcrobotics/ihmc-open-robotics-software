% READIMAGEJLUTFILE Reads lut colourmap file as used by ImageJ
%
% Usage: map = readimagejlutfile(fname)
%
% Argument:  fname - Filename of a .lut file
% Returns:     map - 256 x 3 colourmap table 
%
% The format of a lookup table for ImageJ is 256 bytes of red values, followed
% by 256 green values and finally 256 blue values.  A total of 768 bytes.
%
% See also: MAP2IMAGEJLUTFILE, MAP2ERMAPPERLUTFILE

% 2014 Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au

% PK June 2014

function map = readimagejlutfile(fname)

    map = [];
    
    if ~exist('filename', 'var')
        [filename, pathname] = uigetfile('*.lut');
        if ~filename
            return;
        end
        filename = [pathname filename];
    end
    
    [fid, msg] = fopen(fname, 'r');
    if fid == -1
        error(sprintf('Unable to open %s', fname));
    end
    
    map = fread(fid, inf, 'uint8');
    if length(map) ~= 768
        error('LUT file does not have 768 entries');
    end
    
    map = reshape(map, 256, 3);
    map = double(map)/255;
    
    fclose(fid);