%
%
%
function [filteredCloud index]= filterRandomSample(cloud, ratio)
    nPoints=size(cloud,2);
    index=randperm(nPoints);
    index=index(1:floor(ratio*nPoints));
    filteredCloud = cloud(:,index);
