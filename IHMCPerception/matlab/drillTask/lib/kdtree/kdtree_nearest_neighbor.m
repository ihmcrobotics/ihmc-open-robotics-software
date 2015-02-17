% KDTREE_NEAREST_NEIGHBOR query a kd-tree for nearest neighbor
%
% SYNTAX
% idxs = kdtree_nearest_neighbor( tree, P )
%
% INPUT PARAMETERS
%   tree: a pointer to the previously constructed k-d tree
%   P: a set of N k-dimensional points stored in a 
%      NxK matrix (i.e. each row is a point). For each of these
%      points a kd-tree query is executed and the index
%      of the closest point is stored in the n-th position in the
%      output
%
% OUTPUT PARAMETERS
%   idxs: a column vector of scalars that index the point database.
%         In k-th position the index of the point in the database
%         closest to P(k,:) can be found.
% 
% See also:
% KDTREE_BUILD, KDTREE_NEAREST_NEIGHBOR_DEMO
% 
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
