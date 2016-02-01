% kdtree_k_nearest_neighbors crash when passed with only one argument
clc, clear, close all;
rand('twister',1);
p = rand( 250, 2 );
hold on; xlim( [0 1] ); ylim( [0 1] ); axis equal;
hdata = plot(p(:,1),p(:,2), '.b');
tree = kdtree_build( p );

% range
range = [ [.3 .5]; [.7 .6] ]';
idxs = kdtree_range_query( tree, range );
hrange = plot(p(idxs,1), p(idxs,2), 'or');
line( range(1,[1,2]), range(2,[1,1]) ); % lower 
line( range(1,[1,2]), range(2,[2,2]) ); % upper
line( range(1,[1,1]), range(2,[1,2]) ); % left
line( range(1,[2,2]), range(2,[1,2]) ); % right

% ball
qpoint = [.2;.2]; 
qradii = .2;
idxs = kdtree_ball_query( tree, qpoint, qradii);
circle = zeros( 0,2 );
theta = linspace(0,2*pi,100);
for i=1:100
    [x,y] = pol2cart( theta(i),1 );
    circle(end+1,:) = [x*qradii+qpoint(1),y*qradii+qpoint(2)]; %#ok<AGROW>
end
hball = plot(p(idxs,1), p(idxs,2), '+g');
line( circle(:,1), circle(:,2), 'color', 'g' );

% knn
q = [.8,.2];
plot( q(1), q(2), '.g' );
idxs = kdtree_k_nearest_neighbors(tree, q, 7);
hknn = plot(p(idxs,1), p(idxs,2),'og');
legend('database', 'query', 'query results');
for i=1:numel(idxs)
    text(p(idxs(i),1), p(idxs(i),2),sprintf(' %d',i));
end

legend([hdata, hrange, hball, hknn], 'dataset', 'range query', 'ball query', 'knn query results');

% cleanup
kdtree_delete( tree );