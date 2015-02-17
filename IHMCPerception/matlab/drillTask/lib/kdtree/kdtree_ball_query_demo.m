clc, clear, close all;

%% compile
% mex kdtree_build.cpp
% mex kdtree_ball_query.cpp
% disp('compiled');

%% create data and query
p = rand( 1000, 2 );
tree = kdtree_build(p);
qpoint = [.2;.2]; 
qradii = .2;
[idxs, dists] = kdtree_ball_query( tree, qpoint, qradii);

circle = zeros( 0,2 );
theta = linspace(0,2*pi,100);
for i=1:100
    [x,y] = pol2cart( theta(i),1 );
    circle(end+1,:) = [x*qradii+qpoint(1),y*qradii+qpoint(2)]; %#ok<AGROW>
end

%% visualize
close all;
hold on; xlim( [0 1] ); ylim( [0 1] ); axis equal;
plot(p(:,1),p(:,2), '.b');
plot(p(idxs,1), p(idxs,2), 'or');
line( circle(:,1), circle(:,2) );

%% show labels
for i=1:length(idxs)
    text(p(idxs(i),1), p(idxs(i),2), sprintf('%d', idxs(i) ));
end

% compare distances against computed
matdist = zeros( length(idxs),1 );
for i=1:length(idxs)
    matdist(i) = sqrt( sum((qpoint'-p(idxs(i),:)).^2) );
end
figure(2);
subplot 211, plot( matdist );
subplot 212, plot( dists );
