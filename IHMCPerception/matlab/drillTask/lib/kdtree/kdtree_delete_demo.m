% KDTREE_BUILD_DEMO: illustrate the functionalities of kdtree_delete
clc, clear, close all;

mex kdtree_build.cpp
mex kdtree_delete.cpp
disp('compiled.');

p = rand( 10, 3 );
tree = kdtree_build( p );
disp(sprintf('tree structure pointer: %d',tree ));
disp('deleting tree');

kdtree_delete(tree);

disp('tree deleted correctly');