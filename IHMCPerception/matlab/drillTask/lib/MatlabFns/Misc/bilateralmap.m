% BILATERALMAP Generates a bilateral colourmap
%
% This function generate a colourmap where the first half has one hue and the
% second half has another hue. Saturation varies linearly from 0 in the middle
% to 1 at ech end.  This gives a colourmap which varies from white at the middle
% to an increasing saturation of the different hues as one moves to the ends.
% This colourmap is useful where your data has a clear origin.  The hue
% indicates the polarity of your data, and saturation indicate amplitude.
%
% Usage: map = bilateralmap(H1, H2, V, N)
%
% Arguments:
%           H1 - Hue value for 1st half of map. This must be a value between
%                0 and 1, defaults to 0.65 (blue).
%           H2 - Hue value for 2nd half of map, defaults to 1.0 (red).
%            V - Value as in 'V' in 'HSV', defaults to 1. Reduce this if you
%                want a darker map.
%            N - Number of elements in colourmap, defaults to 256.
%
% Returns:
%         map - N x 3 colourmap of RGB values.
%
% Some nominal hue values:
%  0    - red
%  0.07 - orange
%  0.17 - yellow
%  0.3  - green
%  0.5  - cyan
%  0.65 - blue
%  0.85 - magenta
%  1.0  - red
%
% See also: LABMAP, HSVMAP, GRAYMAP, RANDMAP

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

% October 2012

function map = bilateralmap(H1, H2, V, N)
    
    % Default colourmap values
    if ~exist('H1','var'), H1 = 0.65; end  % blue
    if ~exist('H2','var'), H2 = 1.00; end  % red
    if ~exist('V' ,'var'), V  = 1;    end  % 'value' of 1
    if ~exist('N', 'var'), N  = 256;  end 
    
    % Construct map in HSV then convert to RGB at end
    Non2 = round(N/2);
    map = zeros(N,3);
    
    % First half of map has hue H1 and 2nd half H2
    map(1:Non2, 1) = H1;
    map(1+Non2 : end, 1) = H2;

    % Saturation varies linearly from 0 in the middle to 1 at each end
    map(1:Non2, 2) = (Non2-1:-1:0)'/Non2;
    map(1+Non2 : end, 2) = (1:N-Non2)'/(N-Non2);
    
    % Value is constant throughout
    map(:,3) = V;
    
    map = hsv2rgb(map);