% GENERATELABSLICE Generates RGB image of slice through CIELAB space
%
% Usage: rgbim = generatelabslice(L, flip);
%
% Arguments:    L - Desired lightness level of slice through CIELAB space
%            flip - If set to 1 the image is fliped up-down.  Default is 0
% Returns:  rgbim - RGB image of slice
%
% The size of the returned image is 255 x 255.  
% The achromatic point being at (128, 128)
% To convert from image (row, col) values to CIELAB (a, b) use:
% The a coordinate corresponds to  (col - 128)
% If flip == 0  b =  (row - 128) 
% else          b = -(row - 128)
%
% See also: VIEWLABSPACE

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

% PK September 2014

function rgb = generatelabslice(L, flip)

    if ~exist('flip', 'var'), flip = 0; end
    
    % Define a*b* grid for image
    [a, b] = meshgrid([-127:127]);
    
    if flip
        a = flipud(a);   % Flip to make image display match convention
        b = flipud(b);   %  of LAB space display
    end
    
    [rows,cols] = size(a);
    
    % Build image in lab space
    labim = zeros(rows,cols,3);    
    labim(:,:,1) = L;
    labim(:,:,2) = a;
    labim(:,:,3) = b;
            
    % Generate rgb values from lab
    rgb = applycform(labim, makecform('lab2srgb'));
    
    % Invert to reconstruct the lab values
    labim2 = applycform(rgb, makecform('srgb2lab'));
            
    % Where the reconstructed lab values differ from the specified values is
    % an indication that we have gone outside of the rgb gamut.  Apply a
    % mask to the rgb values accordingly
    mask = max(abs(labim-labim2),[],3);
    
    for n = 1:3
        rgb(:,:,n) = rgb(:,:,n).*(mask<1);  % tolerance of 1
    end
