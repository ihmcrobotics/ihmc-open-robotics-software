addlib
clear
node = rosmatlab.node('dataCollect','http://WuNest:11311');
subCloud = rosmatlab.subscriber('/multisense/image_points2_color', 'sensor_msgs/PointCloud2', 1, node);
%subImage = rosmatlab.subscriber('/multisense/left/image_rect_color', 'sensor_msgs/Image', 1, node);

%sub.setOnNewMessageListeners(@(m) assignin('base','lastPointCloud',m));
%msg = node.newMessage('std_msgs/String');
%publisher = rosmatlab.publisher('/chatter','std_msgs/String',node);

%%
drillCloud = {};

for i=1:5
    [cloud color] = getACloud(subCloud);

    [index] = filterRadius(cloud, 1, [0 0 0]);
    index = filterRandomSample(index, 0.1);
    scatter3(cloud(1,index),cloud(2,index),cloud(3,index), 5, color([3 2 1],index)','fill')
    axis equal
    drawnow

    [~,clickedPoint] = getClickOnPointCloud(gcf,cloud);
    hold off
    index = filterRadius(cloud, 0.3, clickedPoint);
    indexSubsampled = filterRandomSample(index, 0.2);
    scatter3(cloud(1,indexSubsampled),cloud(2,indexSubsampled),cloud(3,indexSubsampled), 5, color(:,indexSubsampled)','fill')
    axis equal
    
    drillCloud{i}=[cloud(:,index);color(:,index)];
    drawnow
end