% RGB2CMYK - Basic conversion of RGB colour table to cmyk 
%
% Usage: cmyk = rgb2cmyk(map)
%
% Argument:    map -  N x 3 table of RGB values (assumed 0 - 1)
% Returns:    cmyk -  N x 4 table of cmyk values
%
% Note that you can use MATLAB's functions MAKECFORM and APPLYCFORM to
% perform the conversion.  However I find that either the gamut mapping, or
% my incorrect use of these functions does not result in a reversable
% CMYK->RGB->CMYK conversion.  Hence this simple function and its companion
% CMYK2RGB
%
% See also: CMYK2RGB, MAP2GEOSOFTTBL, GEOSOFTTBL2MAP

% Peter Kovesi
% Centre for Exploration Targeting 
% The University of Western Australia
% peter.kovesi at uwa edu au

% July 2013
% Feb  2014  % Fix for divide by 0 problem for [0 0 0] 

function cmyk = rgb2cmyk(map)
    
    k = min(1-map,[],2);
    denom = 1 - k + eps;  % Add eps to avoid divide by 0
    c = (1-map(:,1) - k)./denom;
    m = (1-map(:,2) - k)./denom;
    y = (1-map(:,3) - k)./denom;
    
    cmyk = [c m y k];

