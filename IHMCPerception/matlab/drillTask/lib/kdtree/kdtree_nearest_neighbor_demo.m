clc;

%% compile
mex kdtree_build.cpp
mex kdtree_nearest_neighbor.cpp
disp('compiled.');

%% create data and execute query
p = rand( 20, 2 ); % input data
q = rand( 10, 2 ); % query data
tree = kdtree_build( p );
idxs = kdtree_nearest_neighbor(tree,q);

%% visualize
close all;
xlim( [0 1] );
ylim( [0 1] );
hold on; axis equal; axis off;
plot( p(:,1), p(:,2), '.b');
plot(q(:,1), q(:,2),'.r');
plot(p(idxs,1), p(idxs,2),'or');
legend('database', 'query', 'query results');
return;

%% text lables
for i=1:size(q,1)
    text(q(i,1), q(i,2),sprintf(' %d',i));
    text(p(idxs(i),1), p(idxs(i),2),sprintf(' %d',i));
end
