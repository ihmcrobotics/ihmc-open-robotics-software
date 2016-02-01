% GEOSOFTTBL2MAP Converts Geosoft .tbl file to MATLAB colourmap
%
% Usage:  geosofttbl2map(filename, map)
%
% Arguments: filename - Input filename of tbl file
%                 map - N x 3 rgb colourmap
%         
%
% This function reads a Geosoft .tbl file and converts the KCMY values to a RGB
% colourmap.
%
% See also: MAP2GEOSOFTTBL, RGB2CMYK

% Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au

% PK July 2013

function map = geosofttbl2map(filename)
    
    % Read the file
    [fid, msg] = fopen(filename, 'rt');
    error(msg);
    
    % Read, test and then discard first line
    txt = strtrim(fgetl(fid));
    % Very basic file check. Test that it starts with '{'
    if txt(1) ~= '{'
        error('This may not be a Geosoft tbl file')
    end
    
    % Read remaining lines containing the colour table
    [data, count] = fscanf(fid, '%d');
    
    if mod(count,4)  % We expect 4 columns of data
        error('Number of values read not a multiple of 4');
    end

    % Reshape data so that columns form kcmy tuples
    kcmy = reshape(data, 4, count/4);
    % Transpose so that the rows form kcmy tuples and normalise 0-1
    kcmy = kcmy'/255;  
    cmyk = [kcmy(:,2:4) kcmy(:,1)];
    map = cmyk2rgb(cmyk);
    
    fclose(fid);