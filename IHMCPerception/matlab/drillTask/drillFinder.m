
clear
S=load('drillCloud.mat');

%% show data

for i=1:length(S.drillCloud);
    %prepare sample
    cloud = S.drillCloud{i};    
    cloud = filterRandomSample(cloud,0.1);

    %isolate table/drill
    clickedPoint = mean(cloud(1:3,:),2);
    [drillCloud tableCloud ] = findDrill(cloud, 0.01, clickedPoint);
    clf
    hold on
    hDrill=plotCloud(drillCloud);
    hTable=plotCloud(tableCloud);
    hold off
    set(hTable,'MarkerFaceColor','r');

    
    pause(1)
end

%%
     drillCloudA = findDrill(S.drillCloud{1}, 0.01, clickedPoint);
     drillCloudB = findDrill(S.drillCloud{3}, 0.01, clickedPoint);

%% pca on drill
    drillCenter = mean(drillCloud);
    [pc, ~, eigenV]=pca(drillCloud');
    transform


