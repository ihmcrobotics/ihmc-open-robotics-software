% MCLEANUPREGIONS  Morphological clean up of small segments in an image of segmented regions
%
% Usage: [seg, Am] = mcleanupregions(seg, seRadius)
%
% Arguments: seg - A region segmented image, such as might be produced by a
%                  graph cut algorithm.  All pixels in each region are labeled
%                  by an integer.
%       seRadius - Structuring element radius.  This can be set to 0 in which
%                  case  the function will simply ensure all labeled regions
%                  are distinct and relabel them if necessary. 
%
% Returns:   seg - The updated segment image.
%             Am - Adjacency matrix of segments.  Am(i, j) indicates whether
%                  segments labeled i and j are connected/adjacent
%
% Typical application:
% If a graph cut or superpixel algorithm fails to converge stray segments
% can be left in the result.  This function tries to clean things up by:
% 1) Checking there is only one region for each segment label. If there is
%    more than one region they are given unique labels.
% 2) Eliminating regions below the structuring element size
%
% Note that regions labeled 0 are treated as a 'privileged' background region
% and is not processed/affected by the function.
%
% See also: REGIONADJACENCY, RENUMBERREGIONS, CLEANUPREGIONS, MAKEREGIONSDISTINCT

% Copyright (c) 2013 Peter Kovesi
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
% The Software is provided "as is", without warranty of any kind.
%
% March   2013 
% June    2013  Improved morphological cleanup process using distance map

function [seg, Am, mask] = mcleanupregions(seg, seRadius)
option = 2;
    % 1) Ensure every segment is distinct 
    [seg, maxlabel] = makeregionsdistinct(seg);
    
    % 2) Perform a morphological opening on each segment, subtract the opening
    % from the orignal segment to obtain regions to be reassigned to
    % neighbouring segments.
    if seRadius
        se = circularstruct(seRadius);   % Accurate and not noticeably slower
                                         % if radius is small
%       se = strel('disk', seRadius, 4);  % Use approximated disk for speed
        mask = zeros(size(seg));

        if option == 1        
            for l = 1:maxlabel
                b = seg == l;
                mask = mask | (b - imopen(b,se));
            end
            
        else   % Rather than perform a morphological opening on every
               % individual region in sequence the following finds separate
               % lists of unconnected regions and performs openings on these.
               % Typically an image can be covered with only 5 or 6 lists of
               % unconnected regions.  Seems to be about 2X speed of option
               % 1. (I was hoping for more...)
            list = finddisconnected(seg);
            
            for n = 1:length(list)
                b = zeros(size(seg));
                for m = 1:length(list{n})
                    b = b | seg == list{n}(m);
                end

                mask = mask | (b - imopen(b,se));
            end
        end
        
        % Compute distance map on inverse of mask
        [~, idx] = bwdist(~mask);
        
        % Assign a label to every pixel in the masked area using the label of
        % the closest pixel not in the mask as computed by bwdist
        seg(mask) = seg(idx(mask));
    end
    
    % 3) As some regions will have been relabled, possibly broken into several
    % parts, or absorbed into others and no longer exist we ensure all regions
    % are distinct again, and renumber the regions so that they sequentially
    % increase from 1.  We also need to reconstruct the adjacency matrix to
    % reflect the changed number of regions and their relabeling.

    seg = makeregionsdistinct(seg);
    [seg, minLabel, maxLabel] = renumberregions(seg);
    Am = regionadjacency(seg);    
    
