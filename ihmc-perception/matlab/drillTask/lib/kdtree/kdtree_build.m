% KDTREE_BUILD construct a kd-tree from a point cloud
%
% SYNTAX
% tree = kdtree_build(p)
%
% INPUT PARAMETERS
%   P: a set of N k-dimensional points stored in a 
%      NxK matrix. (i.e. each row is a point)
%
% OUTPUT PARAMETERS
%   tree: a pointer to the created data structure
%
% DESCRIPTION
% Given a point set p, builds a k-d tree as specified in [1] 
% with a preprocessing time of O(d N logN), N number of points, 
% d the dimensionality of a point
% 
% See also:
% KDTREE_BUILD_DEMO, KDTREE_NEAREST_NEIGHBOR, 
% KDTREE_RANGE_QUERY, KDTREE_K_NEAREST_NEIGHBORS
%
% References:
% [1] M. De Berg, O. Cheong, and M. van Kreveld. 
%     Computational Geometry: Algorithms and 
%     Applications. Springer, 2008.
%

% Copyright (c) 2008 Andrea Tagliasacchi
% All Rights Reserved
% email: ata2@cs.sfu.ca 
% $Revision: 1.0$  Created on: 2008/09/15
