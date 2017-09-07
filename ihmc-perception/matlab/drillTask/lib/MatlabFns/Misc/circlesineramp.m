% CIRCLESINERAMP Generates test image for evaluating cyclic colour maps
%
% Usage: [im, alpha] = circlesineramp(sze, amp, wavelen, p, hole);
%        [im, alpha] = circlesineramp;
%
% Arguments:     sze - Size of test image.  Defaults to 512x512.
%                amp - Amplitude of sine wave. Defaults to pi/10
%            wavelen - Wavelength of sine wave at half radius of the
%                      circular test image. Defaults to 8 pixels.
%                  p - Power to which the linear attenuation of amplitude, 
%                      from outside edge to centre, is raised.  For no
%                      attenuation use p = 0.  For linear attenuation use a
%                      value of 1.  The default value is 2, quadratic
%                      attenuation. 
%               hole - Flag 0/1 indicating whether the test image should have
%                      a 'hole' in its centre.  The default is 1, to have a
%                      hole, this removes the distraction of the orientation
%                      singularlity at the centre.
% Returns:
%                 im - The test image.
%              alpha - Alpha mask matching the regions outside of of the
%                      circular test image that are set to NaN.  Used if you
%                      want to write an image with these regions transparent.
%
% The test image is a circular pattern consistsing of a sine wave superimposed
% on a spiral ramp function.  The spiral ramp starts at a value of 0 pointing
% right, increasing anti-clockwise to a value of 2*pi as it completes the full
% circle. This gives a 2*pi discontinuity on the right side of the image.  The
% amplitude of the superimposed sine wave is modulated from its full value at
% the outside of the circular pattern to 0 at the centre.  The default sine wave
% amplitude of pi/10 means that the overall size of the sine wave from peak to
% trough represents 2*(pi/10)/(2*pi) = 10% of the total spiral ramp of 2*pi.  If
% you are testing your colour map over a cycle of pi you should use amp = pi/20
% to obtain an equivalent ratio of sine wave to circular ramp.
%
% The image is designed for evaluating the effectiveness of cyclic colour maps.
% It is the cyclic companion to SINERAMP.  Ideally the sine wave pattern should
% be equally discernible over all angles around the test image.  In practice
% many colourmaps have uneven perceptual contrast over their range and often
% include 'flat spots' of no perceptual contrast that can hide significant
% features.  Try MATLAB's hsv colour map.
%
% Ideally the test image should be rendered with a cyclic colour map using
% SHOWANGULARIM though, in this case, rendering the image with SHOW or IMAGESC
% will also be fine because all image values lie within, and use the full range
% of, 0-2*pi.  However, in general, default display methods typically do not
% respect data values directly and can perform inappropriate offsetting and
% normalisation of the angular data before display and rendering with a colour
% map.
%
% For angular data to be rendered correctly it is important that the data values
% are respected so that data values are correctly assigned to specific entries
% in a cyclic colour map.  The assignment of values to colours also depends on
% whether the data is cyclic over pi, or 2*pi.  SHOWANGULARIM supports this.
%
% See also: SHOWANGULARIM, SINERAMP, CHIRPLIN, CHIRPEXP, EQUALISECOLOURMAP, CMAP

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

% September 2014  Original version.
% October   2014  Number of cycles calculated from wave length rather than
%                 being specified directly.

function [im, alpha] = circlesineramp(sze, amp, wavelen, p, hole)
    
    if ~exist('sze','var'),  sze = 512;       end
    if ~exist('amp','var'),  amp = pi/10;     end
    if ~exist('wavelen','var'), wavelen = 8;  end
    if ~exist('p','var'),    p = 2;           end
    if ~exist('hole','var'), hole = 1;        end

    % Set values for inner and outer radii of test pattern
    maxr = sze/2 * 0.9;
    if hole
        minr = 0.15*sze;
    else
        minr = 0;
    end
    
    % Determine number of cycles to achieve desired wavelength at half radius
    meanr = (maxr + minr)/2;
    circum = 2*pi*meanr;
    cycles = round(circum/wavelen);
    
    % Angles are +ve anticlockwise and mod 2*pi
    [x,y] = meshgrid([0:sze-1]-sze/2);
    theta = mod(atan2(-y,x), 2*pi);  
    rad = sqrt(x.^2 + y.^2);
    
    % Normalise radius so that it varies 0-1 over minr to maxr
    rad = (rad-minr)/(maxr-minr);
    
    % Form the image
    im = amp*rad.^p .* sin(cycles*theta) + theta;

    % Ensure all values are within 0-2*pi so that a simple default display
    % with a cyclic colour map will render the image correctly.
    im = mod(im, 2*pi);

    % 'Nanify' values outside normalised radius values of 0-1
    alpha = ones(size(im));
    im(rad > 1) = NaN;  
    alpha(rad > 1) = 0;

    if hole
        im(rad < 0) = NaN; 
        alpha(rad < 0 ) = 0;
    end