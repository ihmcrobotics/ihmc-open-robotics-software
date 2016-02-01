clc;

%% compile
% mex kdtree_build.cpp
% mex kdtree_k_nearest_neighbors.cpp
% disp('compiled.');

%% create data and execute query
rand('seed',1);
p = rand( 30, 2 ); % input data
q = [.5,.5]; % query data
tree = kdtree_build( p );
idxs = kdtree_k_nearest_neighbors(tree,q,10);

%% visualize
close all;
xlim( [0 1] );
ylim( [0 1] );
hold on; axis equal; axis off;
plot( p(:,1), p(:,2), '.b');
plot(q(1), q(2),'.r');
plot(p(idxs,1), p(idxs,2),'or');
legend('database', 'query', 'query results');

%% text lables
for i=1:numel(idxs)
    text(p(idxs(i),1), p(idxs(i),2),sprintf(' %d',i));
end
