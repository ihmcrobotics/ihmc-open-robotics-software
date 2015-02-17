% CMYK2RGB Basic conversion of CMYK colour table to RGB
%
% Usage:  map = cmyk2rgb(cmyk)
%
% Argument:  cmyk -  N x 4 table of cmyk values (assumed 0 - Returns)
% 1:    map -  N x 3 table of RGB values
%
% Note that you can use MATLAB's functions MAKECFORM and APPLYCFORM to
% perform the conversion.  However I find that either the gamut mapping, or
% my incorrect use of these functions does not result in a reversable
% CMYK->RGB->CMYK conversion.  Hence this simple function and its companion
% RGB2CMYK 
%
% See also: RGB2CMYK, MAP2GEOSOFTTBL, GEOSOFTTBL2MAP

% Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au

% PK July 2013

function map = cmyk2rgb(cmyk)
    
    c = cmyk(:,1); m = cmyk(:,2); y = cmyk(:,3); k = cmyk(:,4);

    r = (1-c).*(1-k);
    g = (1-m).*(1-k);
    b = (1-y).*(1-k);
    
    map = [r g b];

