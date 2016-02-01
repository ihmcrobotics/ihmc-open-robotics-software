% MASKIMAGE Apply mask to image
%
% Usage: maskedim = maskimage(im, mask, col)
%
% Arguments:    im  - Image to be masked
%             mask  - Binary masking image
%              col  - Value/colour to be applied to regions where mask == 1
%                     If im is a colour image col can be a 3-vector
%                     specifying the colour values to be applied.
%
% Returns: maskedim - The masked image
%
% See also; DRAWREGIONBOUNDARIES

% Peter Kovesi
% Centre for Exploration Targeting
% School of Earth and Environment
% The University of Western Australia
% peter.kovesi at uwa edu au
%
% Feb 2013

function maskedim = maskimage(im, mask, col)
    
    [rows,cols, chan] = size(im);
    
    % Set default colour to 0 (black)
    if ~exist('col', 'var'), col = 0; end
    
    % Ensure col has same length as image depth.
    if length(col) == 1
        col = repmat(col, [chan 1]);
    else
        assert(length(col) == chan);
    end
    
    % Perform masking
    maskedim = im;
    for n = 1:chan
        tmp = maskedim(:,:,n);
        tmp(mask) = col(n);
        maskedim(:,:,n) = tmp;
    end
    