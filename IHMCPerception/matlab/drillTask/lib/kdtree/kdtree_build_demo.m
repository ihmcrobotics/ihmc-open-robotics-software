%% KDTREE_BUILD_DEMO: illustrates the functionalities of kdtree_build
clc, clear, close all;
mex kdtree_build.cpp 
disp('compiled.');
p = rand( 10, 3 );
tree = kdtree_build( p );
disp(sprintf('tree structure pointer: %d',tree ));
return;

%% KDTREE_BUILD_DEMO2: illustrates performances achieved by the preprocessing
% measure preprocessing time
clc, clear, close all;
N = [1000, 2000, 4000, 8000, 16000, 32000, 64000];
times = zeros( size(N) );
for i=1:length(N)
    disp(i);
    tic
    p = rand( N(i), 3 );
    tree = kdtree_build( p );
    times(i) = toc;   
end
return;