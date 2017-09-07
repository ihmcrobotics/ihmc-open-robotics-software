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
#include <grid_map_lib/iterators/GridMapIterator.hpp>

// Boost
#include <boost/assign/std/vector.hpp>

// ROS
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>

// Eigen
#include <Eigen/Core>

// Math
#include <math.h>

// Random sort
#include <cstdlib>

// Kindr
#include <kindr/thirdparty/ros/RosEigen.hpp>

using namespace std;
using namespace ros;
using namespace foothold_finding_msg;
using namespace boost::assign; // bring 'operator+=()' into scope

namespace foothold_finder {

FootholdFinder::FootholdFinder(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      segmentation_(nodeHandle_)
{
  ROS_INFO("Foothold finder started.");
  readParameters();
  mapRequestTypes_ += "elevation", "variance", "surface_normal_x", "surface_normal_y", "surface_normal_z";
  footholdAdaptService_ = nodeHandle_.advertiseService("adapt", &FootholdFinder::adaptCallback, this);
  elevationMapClient_ = nodeHandle_.serviceClient<grid_map_msg::GetGridMap>(elevationMapServiceName_);
  footShapePublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("foot_shape", 1, true);
  initialize();
}

FootholdFinder::~FootholdFinder() {}

bool FootholdFinder::readParameters()
{
  nodeHandle_.param("elevation_map_service", elevationMapServiceName_, string("/elevation_mapping/get_grid_map"));
  nodeHandle_.param("map_frame", mapFrame_, string("/map"));
  nodeHandle_.param("foothold_evaluation/valid_cell_count_threshold", validCellCountThreshold_, 0.8);
  nodeHandle_.param("foothold_evaluation/max_slope_angle", maxSlopeAngle_, 0.2);

  nodeHandle_.param("foot_shape/line_width", footShapeLineWidth_, 0.01);
  nodeHandle_.param("foot_shape/visualization_offset", footShapeVisualizationOffset_, 0.02);
  std::vector<double> footShapeXCoordinates, footShapeYCoordinates;
  nodeHandle_.getParam("foot_shape/x_coordinates", footShapeXCoordinates);
  nodeHandle_.getParam("foot_shape/y_coordinates", footShapeYCoordinates);

  ROS_ASSERT(footShapeXCoordinates.size() == footShapeYCoordinates.size());
  for(unsigned i = 0; i < footShapeXCoordinates.size(); i++) {
    footShape_.push_back(Eigen::Vector2d(footShapeXCoordinates[i], footShapeYCoordinates[i]));
  }

  nodeHandle_.param("search_region/map_side_length", searchRegionSideLength_, 0.3);
  std::vector<double> searchRegionXCoordinates, searchRegionYCoordinates;
  nodeHandle_.getParam("search_region/x_coordinates", searchRegionXCoordinates);
  nodeHandle_.getParam("search_region/y_coordinates", searchRegionYCoordinates);

  ROS_ASSERT(searchRegionXCoordinates.size() == searchRegionYCoordinates.size());
  for(unsigned i = 0; i < searchRegionXCoordinates.size(); i++) {
    searchRegion_.push_back(Eigen::Vector2d(searchRegionXCoordinates[i], searchRegionYCoordinates[i]));
  }

  segmentation_.readParameters();

  return true;
}

bool FootholdFinder::initialize()
{
  footShapeMarker_.ns = "foot_shape";
  footShapeMarker_.lifetime = ros::Duration(1.0);
  footShapeMarker_.action = visualization_msgs::Marker::ADD;
  footShapeMarker_.type = visualization_msgs::Marker::LINE_STRIP;
  footShapeMarker_.scale.x = footShapeLineWidth_;
  footShapeMarker_.header.frame_id = mapFrame_;
  footShapeMarker_.header.stamp = Time(0.0);

  unsigned int nVertices = footShape_.size() + 1;
  footShapeMarker_.points.resize(nVertices);
  std_msgs::ColorRGBA color;
  color.r = color.g = color.b = color.a = 1.0; // white.
  footShapeMarker_.colors.resize(nVertices, color);

  return true;
  ROS_INFO("Foothold finder initialized.");
  return true;
}

bool FootholdFinder::adaptCallback(foothold_finding_msg::AdaptFootholds::Request& request, foothold_finding_msg::AdaptFootholds::Response& response)
{
  ROS_INFO("Received footholds. Adapting...");

  // TODO Add conversion of position to map frame.
  // TODO Where is the time in the header gone?

  for (const auto& foothold : request.initialFootholds) {
    ROS_INFO("Adapting foothold nr. %i.", foothold.stepNumber);
    Eigen::Vector2d position(foothold.pose.position.x, foothold.pose.position.y);
    grid_map::GridMap map(mapRequestTypes_);
    std::vector<foothold_finding_msg::Foothold> candidates;
    if (getMap(position, searchRegionSideLength_, map)) {
      segmentation_.segment(map);
      findCandidates(foothold, segmentation_, candidates);
    } else {
      foothold_finding_msg::Foothold originalCopy = foothold;
      originalCopy.flag = 0; // unknown
      candidates.push_back(originalCopy);
    }
    response.adaptedFootholds.reserve(response.adaptedFootholds.size() + distance(candidates.begin(), candidates.end()));
    response.adaptedFootholds.insert(response.adaptedFootholds.end(), candidates.begin(), candidates.end());
  }

  ROS_DEBUG("Response: \n -------------------");
  for (const auto& foothold : response.adaptedFootholds) {
    ROS_DEBUG_STREAM(foothold);
  }

  ROS_INFO("Footholds adapted and sent back.");
  return true;
}

bool FootholdFinder::getMap(const Eigen::Vector2d& position, const double sideLength, grid_map::GridMap& map)
{
  ROS_DEBUG("Getting map around position (%f, %f) with side length %f.", position(0), position(1), sideLength);
  // Get map.
  // TODO Could make this general by moving it to grid map package (e.g. createGetGridMapServiceCall(...)).
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

bool FootholdFinder::findCandidates(const foothold_finding_msg::Foothold& foothold, PlaneSegmentation& segmentation, std::vector<foothold_finding_msg::Foothold>& candidates)
{
  ROS_INFO("Finding candidates for foothold nr. %i.", foothold.stepNumber);
  foothold_finding_msg::Foothold candidate = foothold;
  localSearch(segmentation, candidate);
  candidates.push_back(candidate);
  return true;
}

bool FootholdFinder::localSearch(PlaneSegmentation& segmentation, foothold_finding_msg::Foothold& foothold)
{
  // Generate candidate indeces.
  Pose pose;
  kindr::poses::eigen_impl::convertFromRosGeometryMsg(foothold.pose, pose);
  Eigen::Array2i requestedIndex;
  segmentation.getMap()->getIndex(pose.getPosition().toImplementation().head(2), requestedIndex);

  Polygon searchRegion;
  getPolygonInMapFrame(searchRegion_, pose, searchRegion);
  vector<Eigen::Array2i> indices;

  indices.push_back(requestedIndex); // The requested index is twice in the list, but that's ok.
  for (grid_map_lib::PolygonIterator iterator(*segmentation.getMap(), searchRegion);
      !iterator.isPassedEnd(); ++iterator) {
    indices.push_back(*iterator);
  }
  std::random_shuffle((indices.begin()) + 1, indices.end());

  // Search.
  foothold_finding_msg::Foothold candidate;
  unsigned int iIteration = 0;

  for (const auto& index : indices) {
    ROS_DEBUG_STREAM("Search iteration nr. " << iIteration << ".");
    candidate = foothold;
    Eigen::Vector2d position;
    segmentation.getMap()->getPosition(index, position);
    candidate.pose.position.x = position.x();
    candidate.pose.position.y = position.y();
    checkFoothold(segmentation, candidate);
    if (candidate.flag == 2) {
      ROS_DEBUG_STREAM("Valid candidate found.");
      foothold = candidate;
      return true;
    }
    ++iIteration;
  }

  ROS_DEBUG_STREAM("No valid candidate found.");
  return false;
}

// TODO Create class for foothold checking / adaption.
void FootholdFinder::checkFoothold(PlaneSegmentation& segmentation, foothold_finding_msg::Foothold& foothold)
{
  // Get biggest segment in foot area.
  Pose pose;
  kindr::poses::eigen_impl::convertFromRosGeometryMsg(foothold.pose, pose);
  Polygon footShape;
  getPolygonInMapFrame(footShape_, pose, footShape);
  size_t id, nBiggestElements, nValidElements, nElements;
  if (!segmentation.getBiggestSegmentForArea(footShape, id, nBiggestElements, nValidElements, nElements)) {
    foothold.flag = 0;  // unknown.
    return;
  }

  // Visualization.
  publishFootShape(footShape, segmentation.getPlane(id));

  // Adapt foot.
  adapt(segmentation.getPlane(id), foothold); // TODO Move this for performance?

  // Check if foot belongs to the same segment.
  double percentageValid = (double) nBiggestElements / nElements;
  if (percentageValid < validCellCountThreshold_) {
    // TODO Check for different heights!
    ROS_DEBUG_STREAM("Invalid foothold: Too few valid cells in foothold (" << percentageValid << "%).");
    foothold.flag = 3;  // bad.
    return;
  }

  // Check if surface normal is within limits.
  double slopeAngle = segmentation.getSlopeAngle(id);
  if (slopeAngle > maxSlopeAngle_) {
    ROS_DEBUG_STREAM("Invalid foothold: Slope angle " << slopeAngle / M_PI * 180.0 << " is too high.");
    foothold.flag = 3;  // bad.
    return;
  }

  ROS_DEBUG_STREAM("Valid foothold for segment id: " << id << ".");
  foothold.flag = 2;  // verified.
}

void FootholdFinder::adapt(Plane& plane, foothold_finding_msg::Foothold& foothold)
{
  Pose pose;
  kindr::poses::eigen_impl::convertFromRosGeometryMsg(foothold.pose, pose);
  Eigen::Vector2d position(pose.getPosition().getHead(2).toImplementation());

  // Adapt.
  Pose adaptedPose;
  adaptedPose.getPosition().x() = foothold.pose.position.x;
  adaptedPose.getPosition().y() = foothold.pose.position.y;
  adaptedPose.getPosition().z() = plane.getHeight(position);
  adaptedPose.getRotation() = pose.getRotation();
  matchToSurfaceNormal(plane.getNormal(), adaptedPose.getRotation());

  // Fill in data.
  geometry_msgs::Pose poseMessage;
  kindr::poses::eigen_impl::convertToRosGeometryMsg(adaptedPose, poseMessage);
  foothold.pose = poseMessage;
}

// TODO Move this to a more general place.
void FootholdFinder::getPolygonInMapFrame(const Polygon& referencePolygon, const Pose& pose, Polygon& polygonTransformed)
{
  for (const auto& pointInFootFrame : referencePolygon) {
    Eigen::Vector3d point3InFootFrame(pointInFootFrame.x(), pointInFootFrame.y(), 0.0);
    Eigen::Vector3d pointInWorldFrame = pose.getPosition().vector() + pose.getRotation().inverseRotate(point3InFootFrame);
    polygonTransformed.push_back(pointInWorldFrame.head(2));
  }
}

void FootholdFinder::publishFootShape(const Polygon& footShape, Plane& plane)
{
  if (footShapePublisher_.getNumSubscribers () < 1) return;
  ROS_DEBUG("Publishing foot shape for visualization.");
  unsigned i = 0;
  for( ; i < footShape.size(); i++) {
    footShapeMarker_.points[i].x = footShape[i].x();
    footShapeMarker_.points[i].y = footShape[i].y();
    footShapeMarker_.points[i].z = plane.getHeight(footShape[i]) + footShapeVisualizationOffset_;
  }
  footShapeMarker_.points[i].x = footShapeMarker_.points[0].x;
  footShapeMarker_.points[i].y = footShapeMarker_.points[0].y;
  footShapeMarker_.points[i].z = footShapeMarker_.points[0].z;
  footShapePublisher_.publish(footShapeMarker_);
  Duration test(0.01);
  test.sleep();
}

void FootholdFinder::matchToSurfaceNormal(const Eigen::Vector3d& normal, Rotation& rotation)
{
//  ROS_DEBUG_STREAM("Map normal: " << normal.transpose());
//  ROS_DEBUG_STREAM("Rotation before adapting: " << getDebugForm(rotation));
  Eigen::Vector3d reference = Eigen::Vector3d::UnitZ();
  reference = rotation.inverseRotate(reference);
//  ROS_DEBUG_STREAM("Reference normal: " << reference.transpose());
//  double difference = acos(reference.dot(normal) / (reference.norm() * normal.norm())) / M_PI * 180.0;
//  ROS_DEBUG_STREAM("Angle between normal and reference: " << difference << " deg");
  Rotation correction;
  correction.setFromVectors(reference, normal);
//  ROS_DEBUG_STREAM("Correction of adaptation: " << getDebugForm(correction));
  rotation = rotation * correction;
//  ROS_DEBUG_STREAM("Rotation after adapting: " << getDebugForm(rotation));
}

std::string FootholdFinder::getDebugForm(const Rotation& rotation)
{
  kindr::rotations::eigen_impl::EulerAnglesYprPD euler(rotation);
  Eigen::Vector3d eulerVector = euler.getUnique().toImplementation() / M_PI * 180.0;
  return std::string("Yaw: ") + std::to_string(eulerVector(0))
      + " deg, Pitch: " + std::to_string(eulerVector(1))
      + " deg, Roll: " + std::to_string(eulerVector(2)) + " deg";
}

} /* namespace */
