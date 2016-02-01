/*
 * lidar_to_point_cloud_transformer.cpp
 *
 *  Created on: Aug 21, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "lidar_to_point_cloud_transformer/LidarToPointCloudTransformer.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_to_point_cloud_transformer");

  ros::NodeHandle nodeHandle("~");

  lidar_to_point_cloud_transformer::LidarToPointCloudTransformer lidarToPointCloudTransformer(nodeHandle);

  ros::spin();
  return 0;
}
