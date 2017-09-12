% PPDRC Phase Preserving Dynamic Range Compression
%
% Generates a series of dynamic range compressed images at different scales.
% This function is designed to reveal subtle features within high dynamic range
% images such as aeromagnetic and other potential field grids. Often this kind
% of data is presented using histogram equalisation in conjunction with a
% rainbow colourmap. A problem with histogram equalisation is that the contrast
% amplification of a feature depends on how commonly its data value occurs,
% rather than on the amplitude of the feature itself. In addition, the use of a
% rainbow colourmap can introduce undesirable perceptual distortions.
%
% Phase Preserving Dynamic Range Compression allows subtle features to be
% revealed without these distortions. Perceptually important phase information
% is preserved and the contrast amplification of anomalies in the signal is
% purely a function of their amplitude. It operates as follows: first a highpass
% filter is applied to the data, this controls the desired scale of analysis.
% The 2D analytic signal of the data is then computed to obtain local phase and
% amplitude at each point in the image. The amplitude is attenuated by adding 1
% and then taking its logarithm, the signal is then reconstructed using the
% original phase values.
%
% Usage: dim = ppdrc(im, wavelength, clip, savename, n)
%
% Arguments:      im - Image to be processed (can contain NaNs)
%         wavelength - Array of wavelengths, in pixels, of the  cut-in
%                      frequencies to be used when forming the highpass
%                      versions of the image.
%               clip - Percentage of output image histogram to clip.  Only a
%                      very small value should be used, say 0.01 or 0.02, but 
%                      this can be beneficial.  Defaults to 0.01%
%           savename - (optional) Basename of filname to be used when saving
%                      the output images.  Images are saved as
%                      'basename-n.png' where n is the highpass wavelength
%                      for that image .  You will be prompted to select a
%                      folder to save the images in. 
%                  n - Order of the Butterworth high pass filter.  Defaults
%                      to 2
%
% Returns:       dim - Cell array of the dynamic range reduced images.  If
%                      only one wavelength is specified the image is returned 
%                      directly, and not as a one element cell array.
%
% Note when specifying the array 'wavelength' it is suggested that you use
% wavelengths that increase in a geometric series.  You can use the function
% GEOSERIES to conveniently do this
% 
% Example using GEOSERIES to generate a set of wavelengths that increase
% geometrically in 10 steps from 50 to 800. Output is saved in a series of
% image files called 'result-n.png'
%   dim = compressdynamicrange(im, geoseries([50 800], 10), 'result');
%
% View the output images in the form of an Interactive Image using LINIMIX
%
% See also: HIGHPASSMONOGENIC, GEOSERIES, LINIMIX, HISTRUNCATE
%
% Reference:
% Peter Kovesi, "Phase Preserving Tone Mapping of Non-Photographic High Dynamic
% Range Images".  Proceedings: Digital Image Computing: Techniques and
% Applications 2012 (DICTA 2012). Available via IEEE Xplore

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
%
% April 2012  - Original version
% Feb   2014  - Incorporated histogram truncation

function [dim, mask] = ppdrc(im, wavelength, clip, savename, n)

    if ~exist('n', 'var') | isempty(n), n = 2; end
    if ~exist('clip', 'var') | isempty(clip), clip = 0.01; end
    if ~exist('savename', 'var') | isempty(savename), savename = 0; end

    nscale = length(wavelength);
    
    % Identify no-data regions in the image (assummed to be marked by NaN
    % values). These values are filled by a call to fillnan() when passing image
    % to highpassmonogenic.  While fillnan() is a very poor 'inpainting' scheme
    % it does keep artifacts at the boundaries of no-data regions fairly small.
    mask = ~isnan(im);
    [ph, ~, E] = highpassmonogenic(fillnan(im), wavelength, n);
    
    % Construct each dynamic range reduced image 

    if nscale == 1   % Single image:  ph and E will not be cell arrays
        dim = histtruncate(sin(ph).*log1p(E), clip, clip).*mask;
        
    else             % Array of images to be returned.
        range = zeros(nscale,1);
        for k = 1:nscale
            dim{k} = histtruncate(sin(ph{k}).*log1p(E{k}), clip, clip).*mask;
            range(k) = max(abs(dim{k}(:)));
        end
        
        maxrange = max(range);
        % Set the first two pixels of each image to +range and -range so that
        % when the sequence of images are displayed together, say using LINIMIX,
        % there are no unexpected overall brightness changes
        for k = 1:nscale
            dim{k}(1) =  maxrange;
            dim{k}(2) = -maxrange;
        end
    end        

    if savename
        fprintf('Select a folder to save output images in\n');
        dirname = uigetdir([],'Select a folder to save images in');
        if ~dirname, return; end % Cancel
        
        if nscale == 1  
            imwritesc(dim,sprintf('%s/%s-%04d.png', ...
                                     dirname,savename,round(wavelength(1))))
        else
            for k = 1:nscale
                imwritesc(dim{k},sprintf('%s/%s-%04d.png', ...
                                         dirname,savename,round(wavelength(k))))
            end
        end
    end        
    
