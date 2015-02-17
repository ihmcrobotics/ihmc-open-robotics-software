% RENUMBERREGIONS
%
% Usage: [nL, minLabel, maxLabel] = renumberregions(L)
%
% Argument:   L - A labeled image segmenting an image into regions, such as
%                 might be produced by a graph cut or superpixel algorithm.
%                 All pixels in each region are labeled by an integer.
%
% Returns:   nL - A relabeled version of L so that label numbers form a
%                 sequence 1:maxRegions  or 0:maxRegions-1 depending on
%                 whether L has a region labeled with 0s or not.
%      minLabel - Minimum label in the renumbered image.  This will be 0 or 1.
%      maxLabel - Maximum label in the renumbered image.
%
% Application: Segmentation algorithms can produce a labeled image with a non
% contiguous numbering of regions 1 4 6 etc. This function renumbers them into a
% contiguous sequence.  If the input image has a region labeled with 0s this
% region is treated as a privileged 'background region' and retains its 0
% labeling. The resulting image will have labels ranging over 0:maxRegions-1.
% Otherwise the image will be relabeled over the sequence 1:maxRegions
%
% See also: CLEANUPREGIONS, REGIONADJACENCY

% Copyright (c) 2010 Peter Kovesi
% Centre for Exploration Targeting
% School of Earth and Environment
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
% October  2010
% February 2013 Return label numbering range

function [nL, minLabel, maxLabel] = renumberregions(L)

    nL = L;
    labels = unique(L(:))';  % Sorted list of unique labels
    N = length(labels);
    
    % If there is a label of 0 we ensure that we do not renumber that region
    % by removing it from the list of labels to be renumbered.
    if labels(1) == 0
        labels = labels(2:end);
        minLabel = 0;
        maxLabel = N-1;
    else
        minLabel = 1;
        maxLabel = N;
    end
    
    % Now do the relabelling
    count = 1;
    for n = labels
        nL(L==n) = count;
        count = count+1;
    end
    