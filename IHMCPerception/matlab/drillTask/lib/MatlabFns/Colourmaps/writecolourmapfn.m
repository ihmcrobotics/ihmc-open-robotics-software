% WRITECOLOURMAPFN   Creates a MATLAB function file from a Nx3 colourmap 
%
% Usage: writecolourmapfn(map, fname)
%

% PK June 2014

function writecolourmapfn(map, fname)

    [N, chan] = size(map);
    if chan ~= 3
        error('Colourmap must be Nx3');
    end
    
    if strendswith(fname, '.m')     % If fname has been given a .m ending
        fname = strtok(fname, '.'); % remove it (but we will restore it later)
    end
    
    % Convert any '-' characters in the name to '_' as '-' is an operator and
    % cannot be used in a function name
    fname(fname=='-') = '_';
    
    fid = fopen([fname '.m'], 'wt');

    % Strip off any directory path from the filename
    str = strsplit(fname, '/');
    fname = str{end};

    fprintf(fid, '%%  Uniform Perceptual Contrast Colourmap\n');
    fprintf(fid, '%% \n');
    fprintf(fid, '%%  Usage:  map = %s\n', fname);
    fprintf(fid, '%% \n');
    fprintf(fid, '%% \n');
    fprintf(fid, '%%  Centre for Exploration Targeting\n');
    fprintf(fid, '%%  The University of Western Australia\n');
    fprintf(fid, '%%  %s \n', date);
    fprintf(fid, '%% \n');        
    
    fprintf(fid, 'function map = %s\n', fname);
    fprintf(fid, 'map = [ ...\n');

    for n = 1:N
        fprintf(fid, '%f %f %f\n', map(n,1), map(n,2), map(n,3));
    end
    fprintf(fid, '];\n');

    fclose(fid);

    
