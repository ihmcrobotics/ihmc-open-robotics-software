% KDTREE_K_NEAREST_NEIGHBORS query a kd-tree for nearest neighbors
%
% SYNTAX
% idxs = kdtree_k_nearest_neighbors( tree, P, k )
%
% INPUT PARAMETERS
%   tree: a pointer to the previously constructed k-d tree
%   P: a K-dimensional points stored in a Kx1 vector (column)
%   k: the number of closest neighbors to extract 
%
% OUTPUT PARAMETERS
%   idxs: a column vector of scalars that index the point database.
%         the k closest point to P are reported in increasing distance
%         order.
%
% DESCRIPTION
% Given a k-d tree as specified in [1] it computes a k-nearest neighbor
% query (kNN) as specified in [2] with a preprocessing time of O(d N logN)
% and an expected query time of (log N), N number of points, d dimensionality
% of a point in the set.
% 
% See also:
% KDTREE_K_NEAREST_NEIGHBORS_DEMO, KDTREE_BUILD
%
% References:
% [1] M. De Berg, O. Cheong, and M. van Kreveld. 
%     Computational Geometry: Algorithms and 
%     Applications. Springer, 2008.
% [2] J.H. Friedman, J.L. Bentley, R.A. Finkel, "An algorithm
%     for finding best matches in logarithmic expected time,
%     1977, ACM Transactions Math. Softw. pag 209-226
%     DOI: http://doi.acm.org/10.1145/355744.355745
%

% Copyright (c) 2008 Andrea Tagliasacchi
% All Rights Reserved
% email: ata2@cs.sfu.ca 
% $Revision: 1.0$  Created on: 2008/09/15
