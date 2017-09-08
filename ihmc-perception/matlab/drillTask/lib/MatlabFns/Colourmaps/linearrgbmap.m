% LINEARRGBMAP Linear rgb colourmap from black to a specified colour
%
% Usage: map = linearrgbmap(C, N)
%
% Arguments:  C - 3-vector specifying RGB colour
%             N - Number of colourmap elements, defaults to 256
%
% Returns:  map - Nx3 RGB colourmap ranging from [0 0 0] to colour C 
%
% It is suggested that you pass the resulting colour map to EQUALISECOLOURMAP
% to obtain a map with uniform steps in perceptual lightness
% >> map = equalisecolourmap('rgb', linearrgbmap(C, N));
%
% See also: EQUALISECOLOURMAP, TERNARYMAPS, COLOURMAPPATH

% Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au

% PK August 2014

function map = linearrgbmap(C, N)
    
    if ~exist('N', 'var'), N = 256; end
    
    map = zeros(N,3);
    ramp = (0:(N-1))'/(N-1);
    
    for n = 1:3
        map(:,n) = C(n) * ramp;
    end