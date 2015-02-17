% FILTERGRID Generates grid for constructing frequency domain filters
%
% Usage:  [radius, u1, u2] = filtergrid(rows, cols)
%         [radius, u1, u2] = filtergrid([rows, cols])
%
% Arguments:  rows, cols - Size of image/filter
%
% Returns:        radius - Grid of size [rows cols] containing normalised
%                          radius values from 0 to 0.5.  Grid is quadrant
%                          shifted so that 0 frequency is at radius(1,1)
%                 u1, u2 - Grids containing normalised frequency values
%                          ranging from -0.5 to 0.5 in x and y directions
%                          respectively. u1 and u2 are quadrant shifted.
%
% Used by PHASECONGMONO, PHASECONG3 etc etc
%

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
%
% May 2013

function [radius, u1, u2] = filtergrid(rows, cols)

    % Handle case where rows, cols has been supplied as a 2-vector
    if nargin == 1 & length(rows) == 2  
        tmp = rows;
        rows = tmp(1);
        cols = tmp(2);
    end
    
    % Set up X and Y spatial frequency matrices, u1 and u2, with ranges
    % normalised to +/- 0.5 The following code adjusts things appropriately for
    % odd and even values of rows and columns so that the 0 frequency point is
    % placed appropriately.
    if mod(cols,2)
        u1range = [-(cols-1)/2:(cols-1)/2]/(cols-1);
    else
        u1range = [-cols/2:(cols/2-1)]/cols; 
    end
    
    if mod(rows,2)
        u2range = [-(rows-1)/2:(rows-1)/2]/(rows-1);
    else
        u2range = [-rows/2:(rows/2-1)]/rows; 
    end
    
    [u1,u2] = meshgrid(u1range, u2range);
    
    % Quadrant shift so that filters are constructed with 0 frequency at
    % the corners
    u1 = ifftshift(u1);
    u2 = ifftshift(u2);
    
    % Construct spatial frequency values in terms of normalised radius from
    % centre. 
    radius = sqrt(u1.^2 + u2.^2);     
                          
