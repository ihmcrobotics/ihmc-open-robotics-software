% HISTTRUNCATE - Truncates ends of an image histogram.
%
% Function truncates a specified percentage of the lower and
% upper ends of an image histogram.
%
% This operation allows grey levels to be distributed across
% the primary part of the histogram.  This solves the problem
% when one has, say, a few very bright values in the image which
% have the overall effect of darkening the rest of the image after
% rescaling.
%
% Usage: 
%    [newim, sortv] = histtruncate(im, lHistCut, uHistCut)
%    [newim, sortv] = histtruncate(im, lHistCut, uHistCut, sortv)
%
% Arguments:
%    im          -  Image to be processed
%    lHistCut    -  Percentage of the lower end of the histogram
%                   to saturate.
%    uHistCut    -  Percentage of the upper end of the histogram
%                   to saturate.  If omitted or empty defaults to the value
%                   for lHistCut.
%    sortv       -  Optional array of sorted image pixel values obtained
%                   from a previous call to histtruncate.  Supplying this
%                   data speeds the operation of histtruncate when one is
%                   repeatedly varying lHistCut and uHistCut.
%
% Returns:
%    newim       -  Image with values clipped at the specified histogram
%                   fraction values.  If the input image was colour the
%                   lightness values are clipped and stretched to the range
%                   0-1.  If the input image is greyscale no stretching is
%                   applied. You may want to use NORMAALISE to achieve this
%    sortv       -  Sorted image values for reuse in subsequent calls to
%                   histruncate.
%
% See also: NORMALISE

% Copyright (c) 2001-2014 Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% http://www.cet.edu.au/
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.

% July      2001 - Original version
% February  2012 - Added handling of NaN values in image
% February  2014 - Code cleanup
% September 2014 - Default for uHistCut + cleanup

function [newim, sortv] = histtruncate(im, lHistCut, uHistCut, sortv)
    
    if ~exist('uHistCut', 'var') || isempty(uHistCut), uHistCut = lHistCut; end

    if lHistCut < 0 | lHistCut > 100 | uHistCut < 0 | uHistCut > 100
	error('Histogram truncation values must be between 0 and 100');
    end
    
    if ~exist('sortv', 'var'), sortv = []; end

    if ndims(im) == 3  % Assume colour image in RGB
	hsv = rgb2hsv(im);     % Convert to HSV 
        % Apply histogram truncation just to intensity component
        [hsv(:,:,3), sortv] = Ihisttruncate(hsv(:,:,3), lHistCut, uHistCut, sortv);

        % Stretch intensity component to 0-1
        hsv(:,:,3) = normalise(hsv(:,:,3));
	newim = hsv2rgb(hsv);  % Convert back to RGB
    
    else
        [newim, sortv] = Ihisttruncate(im, lHistCut, uHistCut, sortv);
    end
    
    
%-----------------------------------------------------------------------
% Internal function that does the work
%-----------------------------------------------------------------------
    
function [im, sortv] = Ihisttruncate(im, lHistCut, uHistCut, sortv)
    
    if ndims(im) > 2
	error('HISTTRUNCATE only defined for grey value images');
    end
    
    % Generate a sorted array of pixel values or use supplied values
    if isempty(sortv)
        sortv = sort(im(:));
    end
    
    % Any NaN values end up at the end of the sorted list. We need to
    % eliminate these
    sortv = sortv(~isnan(sortv));
    N = length(sortv(:));
    
    % Compute indicies corresponding to specified upper and lower fractions
    % of the histogram.
    lind = floor(1 + N*lHistCut/100);
    hind =  ceil(N - N*uHistCut/100);

    low_in  = sortv(lind);
    high_in = sortv(hind);

    % Adjust image
    im(im < low_in) = low_in;
    im(im > high_in) = high_in;
    
    % Normalise?  normalise with NaNs?
    

    
    
