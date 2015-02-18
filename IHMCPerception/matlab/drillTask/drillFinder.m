
clear
S=load('drillCloud2.mat');
addlib
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
    title(i)
    hold off
    set(hTable,'MarkerFaceColor','r');

    
    pause(1)
end

%%
     indexA=1;
     indexB=2;
     drillCloudA = findDrill(S.drillCloud{indexA}, 0.01, mean(S.drillCloud{indexA}(1:3,:),2));
     drillCloudB = findDrill(S.drillCloud{indexB}, 0.01, mean(S.drillCloud{indexB}(1:3,:),2));
     clf
     hold on
     hA=plotCloud(drillCloudA,1);
     hB=plotCloud(drillCloudB,1);
     hold off
     %set(hA,'MarkerFaceColor','b');
     %set(hB,'MarkerFaceColor','r');

%% pca on drill
    drillCenter = mean(drillCloud);
    [pc, ~, eigenV]=pca(drillCloud');
    transform

%% icp
clf
cloudA=filterRandomSample(drillCloudA,1);
cloudB=filterRandomSample(drillCloudB,1);

subplot(1,2,1);
hold on
hA=plotCloud(cloudA);
set(hA,'MarkerFaceColor','b');
hB=plotCloud(cloudB);
set(hB,'MarkerFaceColor','r');
hold off


%%
[rot trans err]=icp(cloudA(1:3,:),cloudB(1:3,:),50,'Verbose',true,'Minimize','plane');
cloudB2 = [bsxfun(@plus, rot*cloudB(1:3,:), trans);cloudB(4:6,:)];

subplot(1,2,2)
hold on
hA2=plotCloud(cloudA);
set(hA2,'MarkerFaceColor','b');
hB2=plotCloud(cloudB2);
set(hB2,'MarkerFaceColor','r');
hold off
