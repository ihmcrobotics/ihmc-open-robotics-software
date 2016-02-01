% KDTREE_BALL_QUERY query a kd-tree with a ball
%
% SYNTAX
% idxs = kdtree_ball_query(tree, qpoint, qradii)
% [idxs, distances] = kdtree_ball_query(tree, qpoint, qradii);
% 
% INPUT PARAMETERS
%   tree:   a pointer to a valid kdtree structure
%   qpoint: a k-dimensional point speficying the center of the ball
%   qradii: a scalar representing the radius of the ball
% 
% OUTPUT PARAMETERS
%   idxs: a column vector of scalars that index the point database.
%         All the index of points that satisfy the ball query are
%         reported in idxs. No particular ordering is provided
% 
% 	distances: the dinstances from the query result points 
%              to the query point (optional) 
%
% DESCRIPTION
% The ball query is implemented as simple generalization
% of the range query. A range query which inscribes the sphere
% in each dimension is made to the kd-tree, then, the points
% are checked against the distance requirement from the query
% point. A more efficient implementation can be sought.
%
% See also:
% KDTREE_BALL_QUERY_DEMO, KDTREE_BUILD, KDTREE_RANGE_QUERY
%
% References:
% [1] M.De Berg, O.Cheong, and M.van Kreveld. 
%     Computational Geometry: Algorithms and 
%     Applications. Springer, 2008.
%

% Copyright (c) 2008 Andrea Tagliasacchi
% All Rights Reserved
% email: ata2@cs.sfu.ca 
% $Revision: 1.0$  Created on: 2008/09/15
