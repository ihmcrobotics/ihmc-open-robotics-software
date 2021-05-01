Documentation for GDXObstacleDetectionUI:

This is the visualization class for java, it should cooperate with another C++ code to obtain the bounding box,
road point cloud and obstacle point cloud. For C++ code, please refer to https://github.com/Joechencc/lidar_obstacle_detection/blob/main/README.md

The code subscribes to the following topics:

/TunningParam : Tunning parameters for external control (communication completed, but not setup with actual parameters yet)

/boxes : /boxes customerized message topics include all bounding box calculated and expressed in two diagonal points (minPoint, maxPoint)

/downsampled_cloud_obstacle : Downsampled pointcloud for obstacle

/downsampled_cloud_road : Downsampled pointcloud for road


Running step is similiar to other JAVA class

Note that Tunning parameter is setup and sent. Not fully subscribe to real parameters
