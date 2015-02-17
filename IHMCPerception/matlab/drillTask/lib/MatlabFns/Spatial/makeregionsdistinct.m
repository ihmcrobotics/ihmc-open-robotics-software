% MAKEREGIONSDISTINCT Ensures labeled segments are distinct
%
% Usage: [seg, maxlabel] = makeregionsdistinct(seg, connectivity)
%
% Arguments: seg - A region segmented image, such as might be produced by a
%                  superpixel or graph cut algorithm.  All pixels in each
%                  region are labeled by an integer.
%   connectivity - Optional parameter indicating whether 4 or 8 connectedness
%                  should be used.  Defaults to 4.
%
% Returns:   seg - A labeled image where all segments are distinct.
%       maxlabel - Maximum segment label number.
%
% Typical application: A graphcut or superpixel algorithm may terminate in a few
% cases with multiple regions that have the same label.  This function
% identifies these regions and assigns a unique label to them.
%
% See also: SLIC, CLEANUPREGIONS, RENUMBERREGIONS

% Copyright (c) 2013 Peter Kovesi
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

% June 2013


function [seg, maxlabel] = makeregionsdistinct(seg, connectivity)
    
    if ~exist('connectivity', 'var'), connectivity = 4; end
    
    % Ensure every segment is distinct but do not touch segments 
    % with a label of 0
    labels = unique(seg(:))';
    maxlabel = max(labels);
    labels = setdiff(labels,0);  % Remove 0 from the label list
    
    for l = labels
        [bl,num] = bwlabel(seg==l, connectivity);  
        
        if num > 1  % We have more than one region with the same label
            for n = 2:num
                maxlabel = maxlabel+1;  % Generate a new label
                seg(bl==n) = maxlabel;  % and assign to this segment
            end
        end
    end

