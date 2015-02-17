clc, clear, close all;

%% compile
mex kdtree_build.cpp
mex kdtree_nearest_neighbor.cpp
mex kdtree_range_query.cpp
disp('compiled');

%% create data and query
p = rand( 1000, 2 );
tree = kdtree_build(p);
range = [ min( p(1:5,:) ); max( p(1:5,:) ) ]';
idxs = kdtree_range_query( tree, range );

%% visualize
hold on; xlim( [0 1] ); ylim( [0 1] ); axis equal;
plot(p(:,1),p(:,2), '.b');
plot(p(idxs,1), p(idxs,2), 'or');
legend('database', 'range query result');
line( range(1,[1,2]), range(2,[1,1]) ); % lower 
line( range(1,[1,2]), range(2,[2,2]) ); % upper
line( range(1,[1,1]), range(2,[1,2]) ); % left
line( range(1,[2,2]), range(2,[1,2]) ); % right
