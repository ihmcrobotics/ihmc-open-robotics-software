% TERNARYCOLOURS Determine 3 basis colours for a ternary image
%
% This function determines 3 basis colours for constructing ternary/radiometric
% images.  Ideally these are isoluminant and have the same chroma.  While this
% function does not achieve this perfectly the three colours it generates are
% fairly good.
%
% Usage: [C1, C2, C3, rgb1, rgb2, rgb3] = ternarycolours
%
% Returns:   C1 C2 C3 - Basis colours defined in CIELAB space
%              rgb1-3 - Basis colours in RGB space.
%
% The strategy is to first define 2 colours in CIELAB, a nominal 'red' and
% 'green'.  From this we compute third colour, 'blue' so that the 3 colours when
% summed in RGB space produce a neutral grey.
%
% After some experimentation the two colours in CIELAB that I defined were
%     C1 = [53  80  67];   % Red (as in RGB red)
%     C2 = [52 -57  55];   % Green having maximum chroma at a lightness of 52
%
% This results in the third colour being
%     C3 = [50  30 -78];
%
% These are not quite isoluminant but seems to be a good compromise. Choosing a
% green with a lightness of 52 results in a 'blue' with a lightness of about
% 50. While this is slightly darker than the other colours it allows the blue to
% have a larger chroma/saturation than what would be otherwise possible at a
% lightness of 53.  The three colours have chroma of 104, 79 and 86
% respectively.  While these are not equal they are comparable.  Visually the
% result appears to be a reasonable compromise.
% 
% The RGB values that these three colours correspond to are:
%   rgb1 = [1.00  0.00  0.00]
%   rgb2 = [0.00  0.57  0.00]
%   rgb3 = [0.00  0.43  1.00]
%
% Inspecting these values we can see that by using a reduced green (RGB green
% has a lightness of about 88) some of this 'greenness' is transferred to the
% 'blue' basis colour increasing its lightness and making it more of a cyan
% colour.
%
% See also: VIEWLABSPACE, LINEARRGBMAP, LABMAPLIB

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

% August 2014

function [C1, C2, C3, rgb1, rgb2, rgb3] = ternarycolours

    % Define the two initial basis colours.  These are what I came up with by
    % experimentation, edit as you wish...
    C1 = [53 80 67];   % Red (as in RGB red)
    C2 = [52 -57 55];  % Green having maximum chroma at a lightness of 52
    
    % Convert to rgb and subtract from [1 1 1] to solve for the rgb values of
    % the 3rd basis colour.
    rgb1 = lab2rgb(C1);
    rgb2 = lab2rgb(C2);
    rgb3 = [1 1 1] - rgb1 - rgb2;
    
    % Convert back to CIELAB space to check for any gamut clipping etc
    C1 = rgb2lab(rgb1);
    C2 = rgb2lab(rgb2);
    C3 = rgb2lab(rgb3);
    
    % Reconstruct rgb3 to check for gamut clipping
    rgb3 = lab2rgb(C3);
    
    % Construct linear rgb colourmaps to these 3 colours so that we can
    % inspect what we have created.
    map1 = linearrgbmap(rgb1);
    map2 = linearrgbmap(rgb2);
    map3 = linearrgbmap(rgb3);
    
    show(sineramp, 11), colormap(map1);
    show(sineramp, 12), colormap(map2);
    show(sineramp, 13), colormap(map3);
    
    % Form the sum of the colourmaps so that we can check there is no colour
    % cast in the sum of the maps.
    summaps = map1+map2+map3;
    max(summaps(:));
    summaps = summaps/max(summaps(:));
    show(sineramp, 14), colormap(summaps);
    
    figure(15)
    plot(0:255,summaps(:,1), 'r-', ...
         0:255,summaps(:,2), 'g-', ...
         0:255,summaps(:,3), 'b-')
    title('R G B values of summed colourmaps')

