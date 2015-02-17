% RANDMAP Generates a colourmap of random colours
%
% Useful for displaying a labeled segmented image
%
% map = randmap(N)
%
% Argument:   N - Number of elements in the colourmap. Default = 1024.  
%                 This ensures images that have been segmented up to 1024
%                 regions will (well, are more likely to) have a unique
%                 colour for each region. 
%
% See also: HSVMAP, LABMAP, GRAYMAP, BILATERALMAP, HSV, GRAY

% Peter Kovesi
% Centre for Exploration Targeting
% The University of Western Australia
% peter.kovesi at uwa edu au

% February 2013

function map = randmap(N)
    
    if ~exist('N', 'var'),  N = 1024;  end
    map = rand(N, 3);
    map(1,:) = [0 0 0];  % Make first entry black