/*
 * foothold_finder_node.cpp
 *
 *  Created on: Sep 17, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "foothold_finder/FootholdFinder.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "foothold_finder");

  ros::NodeHandle nodeHandle("~");

  foothold_finder::FootholdFinder footholdFinder(nodeHandle);

  ros::spin();
  return 0;
}
