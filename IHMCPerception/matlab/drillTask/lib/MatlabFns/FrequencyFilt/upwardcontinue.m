% UPWARDCONTINUE  Upward continuation for magnetic or gravity potential field data
%
% Usage: [up, pim, psf] = upwardcontinue(im, h, dx, dy)
%
% Arguments:  im - Input potential field image
%              h - Height to upward continue to (+ve)
%         dx, dy - Grid spacing in x and y.  The upward continuation height
%                  is computed relative to the grid spacing.  If omitted dx =
%                  dy = 1, that is, the value of h is in grid spacing units.
%                  If dy is omitted it is assumed dy = dx. 
%
% Returns:    up - The upward continued field image
%            pim - The periodic component of the input potential field
%                  image. See note below.
%            psf - The point spread function corresponding to the upward
%                  continuation height.
%
% Upward continuation filtering is done in the frequency domain whereby the
% Fourier transform of the upward continued image F(Up) is obtained from the
% Fourier transform of the input image F(U) using
%      F(Up) = e^(-2*pi*h * sqrt(u^2 + v^2)) * F(U)
% where u and v are the spatial frequencies over the input grid.
%
% To minimise edge effect problems Moisan's Periodic FFT is used.  This avoids
% the need for data tapering.  Moisan's "Periodic plus Smooth Image
% Decomposition" decomposes an image into two components
%         im = p + s
% where s is the 'smooth' component with mean 0 and p is the 'periodic'
% component which has no sharp discontinuities when one moves cyclically across
% the image boundaries. 
%
% Accordingly if you are doing residual analysis you should subtract the upward
% continued image from the periodic component of the input image 'pim' rather
% than from the raw input image 'im'.
%
% References:  
% Richard Blakely, "Potential Theory in Gravity and Magnetic Applications"
% Cambridge University Press, 1996, pp 315-319
%
% L. Moisan, "Periodic plus Smooth Image Decomposition", Journal of
% Mathematical Imaging and Vision, vol 39:2, pp. 161-179, 2011.
%
% See also: PERFFT2

% Copyright (c) Peter Kovesi
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
%
% June 2012 - Original version
% June 2014 - Tidied up and documented

function  [up, pim, psf] = upwardcontinue(im, h, dx, dy)

    if ~exist('dx', 'var'), dx = 1;  end
    if ~exist('dy', 'var'), dy = dx; end
    
    [rows,cols,chan] = size(im);
    assert(chan == 1, 'Image must be single channel');

    mask = ~isnan(im);
    
    % Use Periodic FFT rather than data tapering to minimise edge effects.
    [IM,~,pim] = perfft2(fillnan(im)); 
        
    % Generate horizontal and vertical frequency grids that vary from
    % -0.5 to 0.5 
    [u1, u2] = meshgrid(([1:cols]-(fix(cols/2)+1))/(cols-mod(cols,2)), ...
			([1:rows]-(fix(rows/2)+1))/(rows-mod(rows,2)));

    % Quadrant shift to put 0 frequency at the corners. Also, divide by grid
    % size to get correct spatial frequencies
    u1 = ifftshift(u1)/dx; 
    u2 = ifftshift(u2)/dy;

    freq = sqrt(u1.^2 + u2.^2); % Matrix values contain spatial frequency
                                % values as a radius from centre (but
                                % quadrant shifted) 
                                
    % Continuation filter in the frequency domain
    W = exp(-2*pi*h*freq);
    
    % Apply filter to obtain upward continuation
    up = real(ifft2(IM.*W)) .* double(mask);
    
    % Reconstruct the spatial representation of the point spread function
    % corresponding to the upward continuation height
    psf = real(fftshift(ifft2(W)));
    
