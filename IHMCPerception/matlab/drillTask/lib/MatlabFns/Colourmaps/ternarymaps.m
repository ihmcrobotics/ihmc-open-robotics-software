% TERNARYMAPS Returns three basis colour maps for generating ternary images
% 
% Usage:  [Rmap, Gmap, Bmap] = ternarymaps(N);
%
% Argument:  N - Number of elements within the colour maps.  This is optional
%                and defaults to 256
% Returns: 
%   Rmap, Gmap, Bmap - Three colour maps that are nominally red, green
%                      and blue but the colours have been chosen so that
%                      they, and their secondary colours, are closely matched
%                      in lightness.
%
% The colours are not as vivid as the RGB primaries but they produce ternary
% images with consistent feature salience no matter what permutation of
% channel-colour assignement is used.
%
% For the derivation of the three primary colours see:
% http://peterkovesi.com/projects/colourmaps/ColourmapTheory/index.html#ternary
%
% See also: TERNARYIMAGE, EQUALISECOLOURMAP, LINEARRGBMAP, APPLYCOLOURMAP

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

function [Rmap, Gmap, Bmap] = ternarymaps(N)
    
    if ~exist('N', 'var'), N = 256; end
    
    % The three primary colours.  For their derivation see:
    % http://peterkovesi.com/projects/colourmaps/ColourmapTheory/index.html#ternary
    R = [0.90 0.17 0.00];
    G = [0.00 0.50 0.00];
    B = [0.10 0.33 1.00];
    
    Rmap = equalisecolourmap('rgb', linearrgbmap(R, N));
    Gmap = equalisecolourmap('rgb', linearrgbmap(G, N));
    Bmap = equalisecolourmap('rgb', linearrgbmap(B, N));
    