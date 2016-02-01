% IMSPECT - Plots image amplitude spectrum averaged over all orientations.
%
% Usage:  [amp, f, slope] = imspect(im, nbins, lowcut)
%                                          \     /
%                                          optional
% Arguments:
%               im     - Image to be analysed.
%               nbins  - No of frequency bins to use (defaults to 100).
%               lowcut - Percentage of lower frequencies to ignore when
%                        finding line of best fit. This avoids problems
%                        with amplitude spikes at low frequencies. 
%                        (defaults to 2%) .
%
% Returns:
%               amp    - 1D array of amplitudes.
%               f      - Corresponding array of frequencies.
%               slope  - Slope of line of best fit to the log-log data
%
% Be wary of using too many frequency bins.  With more bins there will be 
% fewer elements per bin, or even none, producing a very noisy result.
%
% See also: PERFFT2

% Copyright (c) 2001-2003 Peter Kovesi
% School of Computer Science & Software Engineering
% The University of Western Australia
% http://www.csse.uwa.edu.au/
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.

% July 2001  - original version.
% May  2003  - correction of frequency origin for even and odd size images.
%            - corrections to allow for non-square images.
%            - extra commenting
% August 2014 - Changed to use perfft2 rather than fft2 to minimise edge effects

function [amp, f, slope] = imspect(im, nbins, lowcut)
    
    if nargin < 2
	nbins = 100;            % Default No of frequency 'bins'
    end
    if nargin < 3
	lowcut = 2;             % Ignore lower 2% of curve when finding
    end                         % line of best fit.

    amp = zeros(1, nbins);      % preallocate some memory
    fcount = ones(1, nbins);
    
    mag = fftshift(abs(perfft2(double(im))));  % Amplitude spectrum

    % Generate a matrix 'radius' every element of which has a value
    % given by its distance from the centre.  This is used to index
    % the frequency values in the spectrum.
    [rows, cols] = size(im);    
    [x,y] = meshgrid([1:cols],[1:rows]);

    % The following fiddles the origin to the correct position
    % depending on whether we have and even or odd size.  
    % In addition the values of x and y are normalised to +- 0.5
    if mod(cols,2) == 0
      x = (x-cols/2-1)/cols;
    else
      x = (x-(cols+1)/2)/(cols-1);
    end
    if mod(rows,2) == 0
      y = (y-rows/2-1)/rows;
    else
      y = (y-(rows+1)/2)/(rows-1);
    end

    radius = sqrt(x.^2 + y.^2);

    % Quantise radius to the desired number of frequency bins
    radius = round(radius/max(max(radius))*(nbins-1)) + 1;
    
    % Now go through the spectrum and build the histogram of amplitudes
    % vs frequences.
    for r = 1:rows
	for c = 1:cols
	    ind = radius(r,c); 
	    amp(ind) = amp(ind)+mag(r,c);
	    fcount(ind) = fcount(ind)+1;
	end
    end
    
    % Average the amplitude at each frequency bin. We also add 'eps'
    % to avoid potential problems later on in taking logs.
    amp = amp./fcount + eps;             

    % Generate corrected frequency scale for each quantised frequency bin.
    % Note that the maximum frequency is sqrt(0.5) corresponding to the
    % points in the corners of the spectrum
    f = [1:nbins]/nbins*sqrt(.5); 

    % Plots

    % Find first index value beyond the specified histogram cutoff
    fst = round(nbins*lowcut/100 + 1); 
    
    figure(1), clf
    lw = 2;  % Line width
    fs = 16; % Font size
    plot(f(fst:end),amp(fst:end), 'Linewidth', lw)
    xlabel('frequency'), ylabel('amplitude');
    title('Histogram of amplitude vs frequency');

    figure(2), % clf
    loglog(f(fst:end),amp(fst:end), 'Linewidth', lw)
    xlabel('log frequency', 'Fontsize', fs, 'Fontweight', 'bold');
    ylabel('log amplitude', 'Fontsize', fs, 'Fontweight', 'bold');
    
    % Find line of best fit (ignoring specified fraction of low frequency values) 
    p = polyfit(log(f(fst:end)), log(amp(fst:end)),1);
    slope = p(1);
    y = exp(p(1)*log(f(fst:end)) + p(2));        % log(y) = p(1)*log(f) + p(2)
    hold on
    loglog(f(fst:end), y,'Color',[1 0 0], 'Linewidth', lw)

    n = round(nbins/5);
    text(f(n), amp(n)*2, sprintf('Slope = %.2f',p(1)), ...
         'Fontsize', fs, 'Fontweight', 'bold');
    %    title('Histogram of log amplitude vs log frequency');

    set(gca, 'FontSize', 14);
    set(gca, 'FontWeight', 'bold');    
    set(gca, 'LineWidth', 1.5);    

