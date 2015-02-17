% MAP2QGISSTYLE Writes colour maps to QGIS style file
%
%  Usage: map2qgisstyle(map, mapname, filename)
%
% Arguments:   map - N x 3 colour map, or a cell array of N x 3 colour maps 
%          mapname - String giving the colour map name, or a cell array of
%                    strings
%         filename - File name to write QGIS xml style file to.
%
% See also: WRITEQGISMAPS

% Copyright (c) 2014 Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.

% October 2014


function map2qgisstyle(map, mapname, filename)
    
% Ideally I should construct a DOMnode object and write that out using xmlwrite
% but I do not know enough about the QGIS spec.  Hence this hand rolled file.
    
    % If a single colourmap has been supplied rather than a cell array
    % convert it and its mapname to a single element cell arrays
    if ~iscell(map)
        tmp1 = map;   delete map;
        tmp2 = mapname;  delete mapname;
        map{1} = tmp1;
        mapname{1} = tmp2;
    end
    
    nmaps = length(map);
    
    % Ensure file has a .xml ending
    if ~strendswith(filename, '.xml')     
        filename = [filename '.xml'];
    end
    
    [fid, msg] = fopen(filename, 'wt');
    error(msg);
    
    fprintf(fid,'<!DOCTYPE qgis_style>\n');
    fprintf(fid,'<qgis_style version="1">\n');

    fprintf(fid,'<!-- \n');
    fprintf(fid,'CET Perceptually Uniform Colour Maps\n');
    fprintf(fid,'Peter Kovesi\n');
    fprintf(fid,'Center for Exploration Targeting\n');
    fprintf(fid,'The University of Western Australia\n');
    fprintf(fid,'www.cet.edu.au\n');
    fprintf(fid,'-->\n');
    
    fprintf(fid,'<colorramps>\n');

    for m = 1:nmaps
        % Convert colour map values from doubles in range 0-1 to ints ranging
        % from 0-255
        map{m} = round(map{m}*255);
        [mapelements,~] = size(map{m});
        
        
        fprintf(fid,'<colorramp type="gradient" name="%s">\n', mapname{m});
    
        fprintf(fid,'<prop k="color1" v="%d,%d,%d,255"/>\n', ...
                map{m}(1,1), map{m}(1,2), map{m}(1,3));
    
        fprintf(fid,'<prop k="color2" v="%d,%d,%d,255"/>\n', ...
                map{m}(end,1), map{m}(end,2), map{m}(end,3));
        fprintf(fid,'<prop k="discrete" v="0"/>\n');

        fprintf(fid,'<prop k="stops" v="');
        for n = 2:mapelements-1
            fprintf(fid,'%.3f;%d,%d,%d,255',(n-1)/(mapelements-1), ...
                    map{m}(n,1), map{m}(n,2), map{m}(n,3));
            
            if n < (mapelements-1)
                fprintf(fid,':');
            end
        end
        
        fprintf(fid,'"/>\n');    
        fprintf(fid,'</colorramp>\n');
    
    end % for each map
    
    fprintf(fid,'</colorramps>\n');
    fprintf(fid,'</qgis_style>\n');   
    
    fclose(fid);
