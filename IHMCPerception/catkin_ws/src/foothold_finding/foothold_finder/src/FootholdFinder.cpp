/*
 * FootholdFinder.cpp
 *
 *  Created on: Sep 17, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#include "foothold_finder/FootholdFinder.hpp"

// Grid Map
#include <grid_map_msg/GetGridMap.h>
#include <grid_map_lib/GridMapIterator.hpp>

// Boost
#include <boost/assign/std/vector.hpp>

// ROS
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

// Eigen
#include <Eigen/Core>

// Math
#include <math.h>

// Kindr
#include <kindr/thirdparty/ros/RosEigen.hpp>

// Random sort
#include <cstdlib>

using namespace std;
using namespace ros;
using namespace foothold_finding_msg;
using namespace boost::assign; // bring 'operator+=()' into scope

namespace foothold_finder {

FootholdFinder::FootholdFinder(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  ROS_INFO("Foothold finder started.");
  readParameters();
  mapRequestTypes_ += "elevation", "variance", "surface_normal_x", "surface_normal_y", "surface_normal_z";
  footholdAdaptService_ = nodeHandle_.advertiseService("adapt", &FootholdFinder::adaptCallback, this);
  elevationMapClient_ = nodeHandle_.serviceClient<grid_map_msg::GetGridMap>(elevationMapServiceName_);
  segmentationPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("segmented_map", 1);
  initialize();
}

FootholdFinder::~FootholdFinder() {}

bool FootholdFinder::readParameters()
{
  nodeHandle_.param("elevation_map_service", elevationMapServiceName_, string("/elevation_mapping/get_grid_map"));
  nodeHandle_.param("search_area_side_length", searchAreaSideLength_, 0.6);
  nodeHandle_.param("plane_tolerance", planeTolerance_, 0.05);

  // TODO Parameterize this.
  footShape_.push_back(Eigen::Vector2d(-0.08, -0.15));
  footShape_.push_back(Eigen::Vector2d(-0.08, 0.15));
  footShape_.push_back(Eigen::Vector2d(0.08, 0.15));
  footShape_.push_back(Eigen::Vector2d(0.08, -0.15));

  return true;
}

bool FootholdFinder::initialize()
{
  ROS_INFO("Foothold finder visualization initialized.");
  return true;
}

bool FootholdFinder::adaptCallback(foothold_finding_msg::AdaptFootholds::Request& request, foothold_finding_msg::AdaptFootholds::Response& response)
{
  ROS_INFO("Received footholds. Adapting...");

  for (const auto& foothold : request.initialFoodholds) {
    ROS_INFO("Adapting foothold nr. %i.", foothold.stepNumber);
    Eigen::Vector2d position(foothold.pose.position.x, foothold.pose.position.y);
    grid_map::GridMap map(mapRequestTypes_);
    std::vector<foothold_finding_msg::Foothold> candidates;
    if (getMap(position, searchAreaSideLength_, map)) {
      findCandidates(foothold, map, candidates);


    } else {
      foothold_finding_msg::Foothold originalCopy = foothold;
      originalCopy.flag = 0; // unknown
      candidates.push_back(originalCopy);
    }
    response.adaptedFoodholds.reserve(response.adaptedFoodholds.size() + distance(candidates.begin(), candidates.end()));
    response.adaptedFoodholds.insert(response.adaptedFoodholds.end(), candidates.begin(), candidates.end());
  }

  ROS_INFO("Footholds adapted and send back.");
  return true;
}

bool FootholdFinder::getMap(const Eigen::Vector2d& position, const double sideLength, grid_map::GridMap& map)
{
  ROS_INFO("Getting map around postion (%f, %f) with side length %f.", position(0), position(1), sideLength);
  // Get map.
  grid_map_msg::GetGridMap serviceCall;
  serviceCall.request.positionX = position(0);
  serviceCall.request.positionY = position(1);
  serviceCall.request.lengthX = sideLength;
  serviceCall.request.lengthY = sideLength;
  for (const auto& type : mapRequestTypes_) {
    std_msgs::String stringMessage;
    stringMessage.data = type;
    serviceCall.request.dataDefinition.push_back(stringMessage);
  }
  if (!elevationMapClient_.call(serviceCall)) return false;
  map = grid_map::GridMap(serviceCall.response.gridMap);
  return true;
}

bool FootholdFinder::findCandidates(const foothold_finding_msg::Foothold& foothold, grid_map::GridMap& map,
                                    std::vector<foothold_finding_msg::Foothold>& candidates)
{
  ROS_INFO("Finding candidates for foothold nr. %i.", foothold.stepNumber);
  foothold_finding_msg::Foothold candidate = foothold;
  checkAndAdaptFoothold(map, candidate);
  candidates.push_back(candidate);
  return true;
}

void FootholdFinder::checkAndAdaptFoothold(grid_map::GridMap& map, foothold_finding_msg::Foothold& foothold)
{
  Pose pose;
  kindr::poses::eigen_impl::convertFromRosGeometryMsg(foothold.pose, pose);
  Eigen::Vector2d position(pose.getPosition().getHead(2).toImplementation());
  Eigen::Array2i index;
  map.getIndex(position, index);

  // Check.
  Polygon footShape;
  getFootShapeInWorldFrame(pose, footShape);
  clusterPolygonAreaInMap(footShape, map);

  // Adapt.
  Pose adaptedPose;
  adaptedPose.getPosition().x() = foothold.pose.position.x;
  adaptedPose.getPosition().y() = foothold.pose.position.y;
  adaptedPose.getPosition().z() = (double) map.at("elevation", index);

  Eigen::Vector3d normal;
  map.getVector("surface_normal_", index, normal);
  matchToSurfaceNormal(normal, adaptedPose.getRotation());

  // Fill in data.
  geometry_msgs::Pose poseMessage;
  kindr::poses::eigen_impl::convertToRosGeometryMsg(adaptedPose, poseMessage);
  foothold.pose = poseMessage;
  foothold.flag = 1;  // verified.
}

void FootholdFinder::clusterPolygonAreaInMap(const Polygon& polygon, grid_map::GridMap& map)
{
  // Extend map with segmentation id data.
  if (!map.exists("segmentation_id")) map.add("segmentation_id", Eigen::MatrixXf::Constant(map.getBufferSize()(0), map.getBufferSize()(1), NAN));

  // Gather all valid and unsegmented indeces.
  std::vector<Eigen::Array2i> unsegmentedIndeces;
  for (grid_map_lib::PolygonIterator iterator(map, polygon); !iterator.isPassedEnd(); ++iterator) {
    std::vector<std::string> segmentationIdType;
    segmentationIdType.push_back("segmentation_id");
    if (map.isValid(*iterator) && !map.isValid(*iterator, segmentationIdType)) unsegmentedIndeces.push_back(*iterator);
  }

  // Random sort.
  std::random_shuffle(unsegmentedIndeces.begin(), unsegmentedIndeces.end());

  // Segment: Compare each cell from unsegmentedIndeces with remaining points.
  size_t id = 0;
  vector<Eigen::Array2i>::iterator iterator = unsegmentedIndeces.begin();
  for (; iterator != unsegmentedIndeces.end();) {
    map.at("segmentation_id", *iterator) = id;
    vector<Eigen::Array2i>::iterator innerIterator = iterator + 1;
    for (; innerIterator != unsegmentedIndeces.end();) {
      if (isSameCluster(map, *iterator, *innerIterator)) {
        map.at("segmentation_id", *innerIterator) = id;
        innerIterator = unsegmentedIndeces.erase(innerIterator);
      } else {
        ++innerIterator;
      }
    }

    iterator = unsegmentedIndeces.erase(iterator);
    id++;
  }

  if (segmentationPublisher_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 pointCloud;
    map.toPointCloud(pointCloud, "elevation");
    segmentationPublisher_.publish(pointCloud);
  }
}

void FootholdFinder::clusterMap(const Eigen::Vector2d& position, const double radius, grid_map::GridMap& map)
{
//  // Extend map with segmentation id data.
//  map.add("segmentation_id", Eigen::MatrixXf::Constant(map.getBufferSize()(0), map.getBufferSize()(1), NAN));
//
//  // Gather all valid indeces.
//  std::vector<Eigen::Array2i> unsegmentedIndeces; // Move main index to start?
//  for (grid_map_lib::GridMapIterator iterator(map); !iterator.isPassedEnd(); ++iterator) { // TODO Use circle iterator.
//    if (map.isValid(*iterator)) unsegmentedIndeces.push_back(*iterator);
//  }
//
//  // Random sort.
//  std::random_shuffle(unsegmentedIndeces.begin(), unsegmentedIndeces.end());
//
//  // Segment: Compare each cell from unsegmentedIndeces with remaining points.
//  size_t id = 0;
//  vector<Eigen::Array2i>::iterator iterator = unsegmentedIndeces.begin();
//  for (; iterator != unsegmentedIndeces.end();) {
//    map.at("segmentation_id", *iterator) = id;
//    vector<Eigen::Array2i>::iterator innerIterator = iterator + 1;
//    for (; innerIterator != unsegmentedIndeces.end();) {
//      if (isSameCluster(map, *iterator, *innerIterator)) {
//        map.at("segmentation_id", *innerIterator) = id;
//        innerIterator = unsegmentedIndeces.erase(innerIterator);
//      } else {
//        ++innerIterator;
//      }
//    }
//
//    iterator = unsegmentedIndeces.erase(iterator);
//    id++;
//  }
//
//  if (segmentationPublisher_.getNumSubscribers() > 0) {
//    sensor_msgs::PointCloud2 pointCloud;
//    map.toPointCloud(pointCloud, "elevation");
//    segmentationPublisher_.publish(pointCloud);
//  }
}

void FootholdFinder::getFootShapeInWorldFrame(const Pose& pose, Polygon& footShape)
{
  for (const auto& pointInFootFrame : footShape_) {
    Eigen::Vector3d point3InFootFrame(pointInFootFrame.x(), pointInFootFrame.y(), 0.0);
    Eigen::Vector3d pointInWorldFrame = pose.getPosition().vector() + pose.getRotation().inverseRotate(point3InFootFrame);
    footShape.push_back(pointInWorldFrame.head(2));
  }
}

bool FootholdFinder::isSameCluster(const grid_map::GridMap& map, const Eigen::Array2i& firstIndex, const Eigen::Array2i& secondIndex)
{
  Eigen::Vector3d firstPosition, secondPosition;
  map.getPosition3d("elevation", firstIndex, firstPosition);
  map.getPosition3d("elevation", secondIndex, secondPosition);
  Eigen::Vector3d connectingVector = secondPosition - firstPosition;

  Eigen::Vector3d firstNormal, secondNormal;
  map.getVector("surface_normal_", firstIndex, firstNormal);
  map.getVector("surface_normal_", secondIndex, secondNormal);

  if (computeDistanceToNormalPlance(firstNormal, connectingVector) > planeTolerance_) return false;
  if (computeDistanceToNormalPlance(secondNormal, connectingVector) > planeTolerance_) return false;
  return true;
}

double FootholdFinder::computeDistanceToNormalPlance(const Eigen::Vector3d& normal, const Eigen::Vector3d& offset)
{
  double cosAlpha = offset.dot(normal) / (normal.norm() * offset.norm());
  double angleToPlane = fabs(acos(cosAlpha) - M_PI_2);
  double distanceToPlance = sin(angleToPlane) * offset.norm();
  return distanceToPlance;
}

int FootholdFinder::checkFoothold(const grid_map::GridMap& map, const foothold_finding_msg::Foothold& foothold)
{
  return true;
}

void FootholdFinder::matchToSurfaceNormal(const Eigen::Vector3d& normal, Rotation& rotation)
{
  Eigen::Vector3d reference = Eigen::Vector3d::UnitZ();

}

} /* namespace */
