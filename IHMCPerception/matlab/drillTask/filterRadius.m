function [index filteredCloud]= filterRadius(cloud, radius, center)
nPoints=size(cloud,2);
cloudCentered= cloud-repmat(center(:),1,nPoints);
dist = sqrt(mean(sum(cloudCentered.^2),1));
index = find(dist < radius);
if nargout >1
    filteredCloud=cloud(:,index);
end
