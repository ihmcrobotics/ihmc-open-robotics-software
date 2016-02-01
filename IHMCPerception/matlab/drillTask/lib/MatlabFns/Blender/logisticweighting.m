% LOGISTICWEIGHTING  Weighting function based on the logistics function
% 
% Adaptation of the generalised logistics function for defining the variation of
% a weighting function for blending images
%
% Usage: w = logisticweighting(x, b, R)
% 
% Arguments: x - Value, or array of values at which to evaluate the weighting
%                function.
%            b - Parameter specifying the growth rate of the logistics function.
%                This controls the slope of the weighting function at its
%                midpoint.  Probably most convenient to specify this as a
%                power of 2.
%                b = 0      Perfect linear transition from wmin to wmax. 
%                b = 2^0    Near linear transition from wmin to wmax.
%                b = 2^4    Sigmoidal transition.
%                b = 2^10   Near step-like transition at midpoint.
%            R - 4-vector specifying [xmin, xmax, wmin, wmax] the minimum and
%                maximum weights over the minimum and maximum x values that
%                will be used.   The midpoint of the sigmoidal weighting
%                function will occur at (xmin+xmax)/2 at a value of
%                (wmin+wmax)/2. Note that if an x value outside of this range
%                is supplied the resulting weight will also be outside of the
%                desired range. Defaults to  R = [-1 1 -1 1]
%
% Returns:   w - Weight values for each supplied x-coordinate.
%

% Copyright (c) 2012 Peter Kovesi
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
% May 2012

function w = logisticweighting(x, b, R)
    
    if ~exist('R', 'var'), R = [-1 1 -1 1]; end
    
    b = max(b, 1e-10);  % Constrain b to a small value that does not cause
                        % numerical problems
    
    [xmin xmax wmin wmax] = deal(R(1), R(2), R(3), R(4));
    xHalfRange = (xmax-xmin)/2;
    wHalfRange = (wmax-wmin)/2;
    M = (xmin+xmax)/2; % Midpoint of curve

    % We use a form of the generalised logistics function with asymptotes -A and
    % +A, and growth rate b.
    %
    % W(x)  = A - 2*A/(1 + e^(-b*x))
    %
    % First, given the desired value of b, we solve for the value of A that will
    % generate a generalised logistics curve centred on (0,0) and passing
    % through (-xrange/2, -wrange/2) and (+xrange/2, +wrange/2)
    A = wHalfRange/(1 - 2/(1+exp(-b*xHalfRange)));
    
    % Apply an offset of M to x to shift the curve to the desired position
    % and add a vertical offset to obtain the desired weighting range
    w = A - 2*A./(1 + exp(-b*(x-M))) + (wmin+wHalfRange);