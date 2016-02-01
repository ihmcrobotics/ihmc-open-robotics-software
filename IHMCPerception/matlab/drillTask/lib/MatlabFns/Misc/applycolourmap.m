% APPLYCOLOURMAP Applies colourmap to a single channel image to obtain an RGB result
%
% Usage: rgbim = applycolourmap(im, map, range)
%
% Arguments:   im - Single channel image to apply colourmap to.
%             map - RGB colourmap of size ncolours x 3. RGB values are
%                   floating point values in the range 0-1.
%           range - Optional 2-vector specifying the min and max values in
%                   the image to be mapped across the colour map.  Values
%                   outside this range are mapped to the end points of the
%                   colour map.  If range is omitted, or empty, the full range
%                   of image values are used.
%
% Returns:  rgbim - RGB image of floating point values in the range 0-1.
%                   NaN values in the input image are rendered as black.
%
% This function is used by RELIEF as a base image upon which to apply relief
% shading.  Is is also used by SHOWANGULARIM.
%
% See also: IRELIEF, RELIEF, SHOWANGULARIM

% Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au

% November  2013
% September 2014 - Optional range specification added and speeded up by removing loops!

function rgbim = applycolourmap(im, map, range)
    
    [ncolours,chan] = size(map);
    assert(chan == 3, 'Colourmap must have 3 columns');
    assert(ndims(im) == 2, 'Image must be single channel');
    if ~isa(im,'double'), im = double(im); end
 
    if ~exist('range', 'var') || isempty(range)
        range = [min(im(:)) max(im(:))];
    end
    assert(range(1) < range(2), 'range(1) must be less than range(2)');
    
    [rows,cols] = size(im);
    
    % Convert image values to integers that can be used to index into colourmap
    im = round( (im-range(1))/(range(2)-range(1)) * (ncolours-1) ) + 1;

    mask = isnan(im);
    
    im(mask) = 1;           % Set any Nan entries to 1 and
    im(im < 1) = 1;         % clamp out of range entries.
    im(im > ncolours) = ncolours;
    
    rgbim = zeros(rows,cols,3);
    
    rgbim(:,:,1) = ~mask.*reshape(map(im,1), rows, cols);
    rgbim(:,:,2) = ~mask.*reshape(map(im,2), rows, cols);
    rgbim(:,:,3) = ~mask.*reshape(map(im,3), rows, cols);    
    
    
    
