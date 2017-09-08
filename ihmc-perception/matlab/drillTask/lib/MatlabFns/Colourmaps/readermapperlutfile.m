% READERMAPPERLUTFILE Read ER Mapper LUT colour map file
%
% Usage: [map, name, description] = readermapperlutfile(filename)
%
% Argument:  filename - Filename to read
%           
% Returns:
%               map - N x 3 rgb colour map of values 0-1.
%              name - Name of colour map (if specified in file).
%       description - Description of colour map (if specified in file).
%
% The ER Mapper LUT file format is also used by MapInfo
%
% This function has minimal error checking and assumes the file is reasonably
% 'well formed'.
%
% See also: MAP2ERMAPPERLUTFILE, MAP2IMAGEJLUTFILE

% 2014 Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au

% PK Sept 2014

function [map, name, description] = readermapperlutfile(filename)
    
    map = []; name = ''; description = '';
    
    if ~exist('filename', 'var')
        [filename, pathname] = uigetfile('*.lut');
        if ~filename
            return;
        end
        filename = [pathname filename];
    end
    
    [fid,msg] = fopen(filename, 'r');
    if fid == -1
        error(sprintf('Unable to open %s', filename));
    end

    % Read each line, discard empty lines and look for keywords
    line = fgetl(fid);
    [tok, remain] = strtok(line);
    
    while  ~strcmpi(tok, 'LUT')

        if strcmpi(tok, 'Name')
            name = processline(remain);
        elseif strcmpi(tok, 'Description')            
            description = processline(remain);
        elseif strcmpi(tok, 'NrEntries')            
            NrEntries = eval(processline(remain));
        end
        
        line = fgetl(fid);
        if line == -1
            error('Unexpected end of file encountered');
        end

        [tok, remain] = strtok(line);        
    end

    if ~exist('NrEntries', 'var')
        error('Unable to determine number of colour map entries')
    end
    
    % If we get to here we expect to have NrEntries lines of the form
    %  EntryNo   R   G   B
    % There may also be a comment at the end of each line so we have to read
    % them one by one.
    % Note that we read the data from the file in row order but MATLAB stores
    % it in column order hence the [4 NrEntries] dimensioning and the
    % subsequent transpose
    map = zeros(4, NrEntries);
    
    for n = 1:NrEntries
        line = fgetl(fid);
        if line == -1
            error('Unexpected end of file encountered');
        end
        
        % Read 4 ints from each line and ignore anything beyond
        [map(:,n), count] = sscanf(line, '%d %d %d %d');
        if count ~= 4
            error('Incorrect number of entries in colour map');
        end        
        
    end
    
    map = map';
    
    % Read the final } and LookUpTable End ?
    
    fclose(fid);
    
    % Remove the first column of map and rescale values 0-1
    map = double(map(:,2:4))/(2^16-1);
    
    
%---------------------------------------------------------
%  Grab the remaining token in a line ignoring white space, tabs, '=' and '"'
function tok = processline(line)
    
    [tok,remain] = strtok(line, [' = "' char(9)]);