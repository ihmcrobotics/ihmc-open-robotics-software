% ORIENTATIONFILTER  Generate orientation selective filterings of an image
%
% Usage:  oim = orientationfilter(im, norient, angoverlap, boost, cutoff, histcut)
%
% Arguments:     im - Image to be filtered.
%           norient - Number of orientations, try 8.
%        angoverlap - Angular bandwidth overlap factor.  A value of 1 gives a 
%                     minimum overlap between adjacent filter orientations
%                     while still providing full coverage of the spectrum.
%                     However, it is often useful to provide greater overlap
%                     between adjacent filter orientations, especially if the
%                     output is to be viewed using the CYCLEMIX image
%                     blender.  This gives smoother transitions between
%                     images.  Try values 1.5 to 2.0 
%             boost - The ratio that high frequency values are boosted
%                     (or supressed) relative to the low frequency values.
%                     Try values in the range 2 - 4 to start with. Use a
%                     value of 1 for no boost. 
%            cutoff - Cutoff frequency of the highboost filter 0 - 0.5, try 0.2
%           histcut - Percentage of the histogram extremes to truncate.  This
%                     is useful in preventing outlying values in the data
%                     from dominating in the image rescaling for display. Try
%                     a small amount, say, 0.01%
%
% Returns:      oim - Cell array of length 'norient' containing the
%                     orientation filtered images.
%
%
% This function is designed to help identify oriented structures within an image
% by applying orientation selective filters.  If one is looking for lineaments
% it is typically useful to boost the high frequency components in the image as
% well, hence the combination of the two filters
%
% Example: Generate 8 orientation filterings of an image with an angular
% bandwidth overlap factor of 1.5 and amplifying spatial frequencies greater
% than 0.1 (wavelenths < 10 pixels) by a factor of 2.  Also truncate the image
% histograms so that 0.01% of image pixels are saturated.  This helps ensure
% that outlying data values do not dominate when the image is scaled for
% display.
%
% >> oim = orienationfilter(im, 8, 1.5, 2, 0.1, 0.01);
%
% An effective way to view all the output images is to use the cyclic image
% blending tool CYCLEMIX.m
%
% >> cyclemix(oim);
%
% Note that the cyclemix control wheel varies from 0 - 2pi and these angles are
% mapped to the orientation filtered images which vary from 0 - pi in angle.
% This makes the interface a bit counterintuitive to use.  To get around this
% one can replicate the the cell array of orientation filtered images giving an
% array that corresponds to the angles 0 - pi, 0 - pi.  When this is passed to
% cyclemix you get a much more intuitive interface
%
% >> cyclemix({oim{:} oim{:}})
% 
% See also: CYCLEMIX, HIGHBOOSTFILTER, HISTTRUNCATE 

% Copyright (c) 2012-2014 Peter Kovesi
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

% August 2012   Original version
% July   2014   Revised to incorporate highboost filtering and histogram truncation

function oim = orientationfilter(im, norient, angoverlap, boost, cutoff, histcut)
    
    IM = fft2(im);
    [rows,cols] = size(im);

    if ~exist('angoverlap','var')
        angScale = norient/2;
    else
        angScale = norient/2 / angoverlap;
    end
    
    % If a highboost filter has been specified apply it to the fourier
    % transform of the image.
    if exist('cutoff', 'var')
        IM = IM .* highboostfilter([rows,cols], cutoff, 1, boost);
    end
    
    if ~exist('histcut','var'), histcut = 0; end
    
    % Construct frequency domain filter matrices
    [ ~ , u1, u2] = filtergrid(rows,cols);
    
    theta = atan2(-u2,u1);            % Matrix values contain polar angle.
                                      % (note -ve y is used to give +ve
                                      % anti-clockwise angles)
    sintheta = sin(theta);
    costheta = cos(theta);

    for o = 1:norient                 % For each orientation...
        angl = (o-1)*pi/norient;           % Filter angle.
        
        % For each point in the filter matrix calculate the angular distance from
        % the specified filter orientation.  To overcome the angular wrap-around
        % problem sine difference and cosine difference values are first computed
        % and then the atan2 function is used to determine angular distance.
        ds = sintheta * cos(angl) - costheta * sin(angl);    % Difference in sine.
        dc = costheta * cos(angl) + sintheta * sin(angl);    % Difference in cosine.
        dtheta = abs(atan2(ds,dc));                          % Absolute angular distance.
      
        % Scale theta so that cosine spread function has the right wavelength and clamp to pi    
        dtheta = min(dtheta*angScale,pi);

        % The orientation filter haa a cosine cross section in the frequency domain
        % The spread function is cos(dtheta) between -pi and pi.  We add 1,
        % and then divide by 2 so that the value ranges 0-1
        spread = (cos(dtheta)+1)/2;        
        spread = spread + fliplr(flipud(spread));
        %        show(fftshift(spread),o);
        
        % Apply orientation filter
        oim{o} = real(ifft2(IM.*spread));
        
        % Truncate extremes of image histogram
        if histcut
            oim{o} = histtruncate(oim{o}, histcut, histcut);
        end
    end