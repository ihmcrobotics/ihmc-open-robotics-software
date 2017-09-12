% KDTREE_RANGE_QUERY query a kd-tree with a range
%
% SYNTAX
% idxs = kdtree_range_query(tree, range)
%
% INPUT PARAMETERS
%   range: a matrix representing range information for the query.
%          in the first column, k-th row the inferior value for the 
%          query (point included) in dimension k is found. Similarly
%          the superior limits are contained in the second column
%
% OUTPUT PARAMETERS
%   idxs: a column vector of scalars that index the point database.
%         All the index of points that satisfy the range query are
%         reported in idxs. No particular ordering is provided
% 
% See also:
% KDTREE_RANGE_QUERY_DEMO, KDTREE_RANGE_QUERY, KDTREE_BUILD,
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
