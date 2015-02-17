addlib
clear
node = rosmatlab.node('dataCollect','http://WuNest:11311');
sub = rosmatlab.subscriber('/multisense/image_points2_color', 'sensor_msgs/PointCloud2', 1, node);
%sub = rosmatlab.subscriber('/chatter', 'std_msgs/String', 1, node);

%sub.setOnNewMessageListeners(@(m) assignin('base','lastPointCloud',m));
%msg = node.newMessage('std_msgs/String');
%publisher = rosmatlab.publisher('/chatter','std_msgs/String',node);

%%
drillCloud = {};

for i=1:10
    [cloud color] = getACloud(sub);

    [index] = filterRadius(cloud, 1, [0 0 0]);
    index = filterRandomSample(index, 0.1);
    scatter3(cloud(1,index),cloud(2,index),cloud(3,index), 5, color([3 2 1],index)','fill')
    axis equal

    [~,clickedPoint] = getClickOnPointCloud(gcf,cloud);
    hold off
    index = filterRadius(cloud, 0.3, clickedPoint);
    indexSubsampled = filterRandomSample(index, 0.2);
    scatter3(cloud(1,indexSubsampled),cloud(2,indexSubsampled),cloud(3,indexSubsampled), 5, color(:,indexSubsampled)','fill')
    axis equal
    
    i=10
    drillCloud{i}=[cloud(:,index);color(:,index)];

end