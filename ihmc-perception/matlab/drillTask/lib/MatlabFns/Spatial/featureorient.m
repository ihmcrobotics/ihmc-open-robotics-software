% FEATUREORIENT - Estimates the local orientation of features in an edgeimage
%
% Usage:  orientim = featureorient(im, gradientsigma,...
%                                      blocksigma, ...
%                                      orientsmoothsigma, ...
%                                      radians)
%
% Arguments:  im                - A normalised input image.
%             gradientsigma     - Sigma of the derivative of Gaussian
%                                 used to compute image gradients.  This can
%                                 be 0 as the derivatives are calculaed with
%                                 a 5-tap filter.
%             blocksigma        - Sigma of the Gaussian weighting used to
%                                 form the local weighted sum of gradient
%                                 covariance data.  A small value around 1,
%                                 or even less, is usually fine on most
%                                 feature images.
%             orientsmoothsigma - Sigma of the Gaussian used to smooth
%                                 the final orientation vector field. 
%                                 Optional: if ommitted it defaults to 0
%             radians           - Optional flag 0/1 indicating whether the
%                                 output should be given in radians. Defaults
%                                 to 0 (degrees)
% 
% Returns:    orientim          - The orientation image in degrees/radians
%                                 range is (0 - 180) or (0 - pi).
%                                 Orientation values are +ve anti-clockwise
%                                 and give the orientation across the feature
%                                 ridges 
%
% The intended application of this function is to compute orientations on a
% feature image prior to nonmaximal suppression in the case where no orientation
% information is available from the feature detection process.  Accordingly 
% the default output is in degrees to suit NONMAXSUP.
%
% See also: NONMAXSUP, RIDGEORIENT

% Copyright (c) 1996-2013 Peter Kovesi
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

% May 2013  Adapted from RIDGEORIENT

function or = featureorient(im, gradientsigma, blocksigma, orientsmoothsigma, ...
                                 radians)
        
    if ~exist('orientsmoothsigma', 'var'), orientsmoothsigma = 0; end
    if ~exist('radians', 'var'), radians = 0; end
    [rows,cols] = size(im);
    
    im = gaussfilt(im, gradientsigma);   % Smooth the image.
    [Gx, Gy] = derivative5(im,'x','y');  % Get derivatives.   
    Gy = -Gy;   % Negate Gy to give +ve anticlockwise angles.
    
    % Estimate the local ridge orientation at each point by finding the
    % principal axis of variation in the image gradients.
    Gxx = Gx.^2;       % Covariance data for the image gradients
    Gxy = Gx.*Gy;
    Gyy = Gy.^2;
    
    % Now smooth the covariance data to perform a weighted summation of the
    % data.
    Gxx =   gaussfilt(Gxx, blocksigma);
    Gxy = 2*gaussfilt(Gxy, blocksigma);
    Gyy =   gaussfilt(Gyy, blocksigma);
    
    % Analytic solution of principal direction
    denom = sqrt(Gxy.^2 + (Gxx - Gyy).^2) + eps;
    sin2theta = Gxy./denom;            % Sine and cosine of doubled angles
    cos2theta = (Gxx-Gyy)./denom;

    if orientsmoothsigma
        cos2theta = gaussfilt(cos2theta, orientsmoothsigma); % Smoothed sine and cosine of
        sin2theta = gaussfilt(sin2theta, orientsmoothsigma); % doubled angles
    end
    
    or = atan2(sin2theta,cos2theta)/2;
    or(or<0) = or(or<0)+pi;             % Map angles to 0-pi.

    if ~radians                         % Convert to degrees.
        or = or*180/pi;   
    end