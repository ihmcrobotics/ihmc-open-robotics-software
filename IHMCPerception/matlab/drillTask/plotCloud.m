function h=plotCloud(cloud, index, pointSize)
    if ~exist('pointSize','var') || isempty(pointSize)
        pointSize=10;
    end
    if ~exist('index','var') || isempty(index)
        nPoint=size(cloud,2);
        index=randperm(nPoint);
        index=index(1:min(5000,nPoint));
    elseif length(index)==1
        nPoint = index;
        index=randperm(nPoint);
        index=index(1:min(5000,nPoint));
    end
    h=scatter3(cloud(1,index),cloud(2,index),cloud(3,index),pointSize,cloud(4:6,index)','fill');
    axis equal
