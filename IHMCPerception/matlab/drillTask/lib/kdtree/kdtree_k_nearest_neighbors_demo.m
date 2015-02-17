% clc, clear, close all
% 
% %% compile
% % mex kdtree_build.cpp
% % mex kdtree_k_nearest_neighbors.cpp
% % disp('necessary files compiled.');
% 
% %% test1: small test for evaluation and visualizatoin
% rand('seed',1);
% p = rand( 30,  2 ); % input data
% q = [ .5, .5 ]'; k = 5; % query data (buggy)
% 
% % execute query by kd-tree and by linear scan
% tree = kdtree_build( p );
% idxs1 = kdtree_k_nearest_neighbors( tree,q,k );
% dist = zeros( size(p,1), 1 );
% for i=1:size(p,1)
%     dist( i ) = sqrt( sum( ( p(i,:)-q' ).^2 ) );
% end
% [IGNORE, idxs2] = sort( dist,'ascend' );
% idxs2 = idxs2(1:k); % keep only first k entries
% 
% %%% visualize indexes returned by query
% disp( ['(fullsearch)   kNN indexes: ', sprintf('%d ', idxs2 ) ] );
% disp( ['(kdtreesearch) kNN indexes: ', sprintf('%d ', idxs1 ) ] );
% 
% %%% visualize
% xlim( [0 1] );
% ylim( [0 1] );
% hold on; axis equal; axis off;
% plot( p(:,1), p(:,2), '.b');
% plot(q(1), q(2),'.r');
% plot(p(idxs2,1), p(idxs2,2),'or');
% plot(p(idxs1,1), p(idxs1,2),'+y');
% legend('database', 'query', '(matlab) result', '(kdtree) result');
% 
% %%% convert input data to C++ format (for testing)
% % for i=1:size(p,1)
% %    disp(sprintf('A[%d][0] = %.2f; A[%d][1] = %.2f;', i-1, p(i,1), i-1, p(i,2) )); 
% % end
% 
% return

%% test2: extensive random performance and correctness test
% clc, clear, close all;
% NUMTESTS = 10;
% NUMQUERS = 100;
% N = 10000;
% counter = 1;
% kdtree_times = zeros( NUMTESTS*NUMQUERS,1 );
% matlab_times = zeros( NUMTESTS*NUMQUERS,1 );
% 
% for j=1:NUMTESTS
%     disp(sprintf('executing test #%d',j));
%     p = rand( 10000,   2 ); % input data
%     q = rand( NUMQUERS,2 ); % query data
%     k = ceil(N/10*rand( 1,1 ));   % query size (number of kNN to extract)
%     
%     %% execute query by kd-tree
%     tree = kdtree_build( p );
%     
%     for m=1:NUMQUERS
%         % kdtree based
%         tic
%         idxs_kdtree = kdtree_k_nearest_neighbors( tree,q(m,:)',k );
%         kdtree_times( counter ) = toc();
%         
%         % matlab based
%         tic
%         dist = zeros( size(p,1), 1 );
%         for i=1:size(p,1)
%             dist( i ) = sqrt( sum( ( p(i,:)-q(m,:) ).^2 ) );
%         end
%         [IGNORE, idxs2] = sort( dist,'ascend' );
%         idxs_matlab = idxs2(1:k);
%         matlab_times( counter ) = toc();
%         
%         if( ~all( idxs_kdtree==idxs_matlab) )
%             error('kdtree gave incorrect results');
%         end
%         
%         counter = counter + 1;
%     end   
% end
% 
% subplot( 211 ), plot( kdtree_times ); ylim( [0,0.05] );
% subplot( 212 ), plot( matlab_times ); ylim( [0,0.05] );
% disp( sprintf('kdtree average time (ms): %d ', mean(kdtree_times)/1000 ) );
% disp( sprintf('matlab average time (ms): %d ', mean(matlab_times)/1000 ) );
% return;

% test 3: check if distances returned are correct!
clc, clear, close all
rand('twister',1);
p = rand( 1000,  2 ); % input data
q = [ .5, .5 ]; 
k = 100;

% execute query by kd-tree and by linear scan
tree = kdtree_build( p );
[idxs1, dists] = kdtree_k_nearest_neighbors( tree,q,k );
dist = zeros( size(p,1), 1 );
for i=1:size(p,1)
    dist( i ) = sqrt( sum( ( p(i,:)-q ).^2 ) );
end
[IGNORE, idxs2] = sort( dist,'ascend' );
idxs2 = idxs2(1:k); % keep only first k entries
disp( sprintf('search is correct?: %d', all(idxs1==idxs2)));

% compare distances against computed
matdist = zeros( length(idxs1),1 );
for i=1:length(idxs1)
    matdist(i) = sqrt( sum((q-p(idxs1(i),:)).^2) );
end
figure(2);
subplot 211, plot( matdist );
subplot 212, plot( dists );

