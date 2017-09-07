% MAP2GEOSOFTTBL Converts MATLAB colourmap to Geosoft .tbl file
%
% Usage:  map2geosofttbl(map, filename, cmyk)
%
% Arguments:   map - N x 3 rgb colourmap
%         filename - Output filename
%             cmyk - Optional flag 0/1 indicating whether CMYK values should
%                    be written.  Defaults to 0 whereby RGB values are
%                    written
%
% This function writes a RGB colourmap out to a .tbl file that can be loaded
% into Geosoft Oasis Montaj
%
% See also: RGB2CMYK

% Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au

% PK October 2012
%    June 2014    - RGB or CMYK option

function map2geosofttbl(map, filename, cmyk)

    if ~exist('cmyk', 'var'), cmyk = 0; end

    [N, cols] = size(map);
    if cols ~= 3
        error('Colourmap must be N x 3 in size')
    end
    
    % Ensure filename ends with .tbl
    if ~strendswith(filename, '.tbl')
        filename = [filename '.tbl'];
    end
    
    fid = fopen(filename, 'wt');    

    if cmyk % Convert RGB values in map to CMYK and scale 0 - 255
        cmyk = round(rgb2cmyk(map)*255);
        kcmy = circshift(cmyk, [0 1]); % Shift K to 1st column
        
        fprintf(fid, '{ blk cyn mag yel }\n');
        for n = 1:N
            fprintf(fid, ' %03d  %03d  %03d  %03d \n', kcmy(n,:));    
        end
        
    else    % Write RGB values
        map = round(map*255);
        
        fprintf(fid, '{ red  grn  blu }\n');
        for n = 1:N
            fprintf(fid, '  %3d  %3d  %3d\n', map(n,:));    
        end        
        
    end
    
    fclose(fid);
    