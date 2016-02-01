/*
 * PlaneSegmentation.cpp
 *
 *  Created on: Oct 16, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "foothold_finder/PlaneSegmentation.hpp"

// Grid Map
#include <grid_map_lib/iterators/GridMapIterator.hpp>

// ROS
#include <sensor_msgs/PointCloud2.h>

// Random sort
#include <cstdlib>

using namespace std;

namespace foothold_finder {

PlaneSegmentation::PlaneSegmentation(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  readParameters();
  typeNameList_.push_back(typeName_);
  visualizationPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("segmented_map", 1);
  detailedVisualizationPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("segmented_map_detailed", 1);
}

PlaneSegmentation::~PlaneSegmentation() {}

bool PlaneSegmentation::readParameters()
{
  nodeHandle_.param("plane_segmentation/distance_tolerance", planeDistanceTolerance_, 0.02);
  nodeHandle_.param("plane_segmentation/angle_tolerance", planeAngleTolerance_, 0.1);

  double sleepDurationBetweenSegmentationSteps;
  nodeHandle_.param("plane_segmentation/sleep_duration_between_segmentation_steps", sleepDurationBetweenSegmentationSteps, 0.0);
  sleepDurationBetweenSegmentationSteps_.fromSec(sleepDurationBetweenSegmentationSteps);

  nodeHandle_.param("plane_segmentation/min_cells_for_segment", minCellsForSegment_, 9);
  nodeHandle_.param("plane_segmentation/max_optimization_steps", maxOptimizationSteps_, 10);
  return true;
}

bool PlaneSegmentation::initializeSegmentation()
{
  unsegmentedIndeces_.clear();
  planes_.clear();

  // Extend map with segmentation id data.
  if (map_->exists(typeName_)) return false;
  map_->add(typeName_, Eigen::MatrixXf::Constant(map_->getBufferSize()(0), map_->getBufferSize()(1), NAN));

  // Gather all valid indeces.
  for (grid_map_lib::GridMapIterator iterator(*map_); !iterator.isPassedEnd(); ++iterator) {
    if (map_->isValid(*iterator)) unsegmentedIndeces_.push_back(*iterator);
  }

  // Random sort.
  std::random_shuffle(unsegmentedIndeces_.begin(), unsegmentedIndeces_.end());

  return true;
}

bool PlaneSegmentation::segment(grid_map::GridMap& map)
{
  map_ = &map;
  if(!initializeSegmentation()) return false;

  // Segment: Compare each cell from unsegmentedIndeces with remaining points.
  size_t id = 0;
  vector<Eigen::Array2i>::iterator iterator;

  for (iterator = unsegmentedIndeces_.begin(), id = 0;
      iterator != unsegmentedIndeces_.end();
      iterator = unsegmentedIndeces_.begin(), id++) {

    // Creating new segment.
    std::vector<Eigen::Array2i> segmentIndeces;
    segmentIndeces.push_back(*iterator);
    unsegmentedIndeces_.erase(iterator);

    // Primary segmentation.
    Plane averagePlane;
    averageWithPlane(*iterator, averagePlane);
    extendSegment(segmentIndeces, id, averagePlane);

    // Optimization / Clean-up.
    bool hasSegmentChanged = true;
    size_t nOptimizationSteps = 0;
    while (hasSegmentChanged && nOptimizationSteps < maxOptimizationSteps_) {

      // Check if segment has minimal size.
      if (segmentIndeces.size() < minCellsForSegment_) {
//        unsegmentedIndeces_.insert(unsegmentedIndeces_.end(), segmentIndeces.begin(), segmentIndeces.end()); // TODO Think about this.
        segmentIndeces.clear();
        break;
      }

      // Removing points that do not match the average plane anymore.
      hasSegmentChanged = false;
      for (vector<Eigen::Array2i>::iterator i = segmentIndeces.begin(); i != segmentIndeces.end();) {
        if (!belongsToSegment(*i, averagePlane)) {
          unsegmentedIndeces_.push_back(*i);
          i = segmentIndeces.erase(i);
          hasSegmentChanged = true;
        } else {
          ++i;
        }
      }

      if (segmentIndeces.size() == 0) break;
      publishForVisualization(segmentIndeces, id);

      // Compute new average plane.
      averagePlane.clear();
      for (const auto& i : segmentIndeces) averageWithPlane(i, averagePlane);
      publishForVisualization(segmentIndeces, id);

      // Check for new correspondences.
      bool areNewSegmentsAdded = extendSegment(segmentIndeces, id, averagePlane);
      hasSegmentChanged = hasSegmentChanged || areNewSegmentsAdded;

      nOptimizationSteps++;
    }

    // Mark segment in map.
    for (const auto& i : segmentIndeces) map_->at("segmentation_id", i) = id;
    planes_.insert(std::pair<unsigned int, Plane>(id, averagePlane));
  }

  publishForVisualization();
  return true;
}

Plane& PlaneSegmentation::getPlane(const size_t& segmentId)
{
  return planes_[segmentId];
}

double PlaneSegmentation::getSlopeAngle(const size_t& segmentId)
{
  return computeAngleBetweenVectors(planes_[segmentId].getNormal(), Eigen::Vector3d::UnitZ());
}

const grid_map::GridMap* PlaneSegmentation::getMap()
{
  return map_;
}

bool PlaneSegmentation::getBiggestSegmentForArea(const Polygon& polygon, size_t& segmentId, size_t& nBiggestElements, size_t& nValidElements, size_t& nElements)
{
  nValidElements = nElements = 0;
  std::map<size_t, size_t> segmentSizes;
  for (grid_map_lib::PolygonIterator iterator(*map_, polygon); !iterator.isPassedEnd(); ++iterator) {
    ++nElements;
    if (!map_->isValid(*iterator, typeNameList_)) continue;
    ++nValidElements;
    size_t id = map_->at(typeName_, *iterator);
    if (segmentSizes.find(id) == segmentSizes.end()) {
      segmentSizes[id] = 1;
    } else {
      ++segmentSizes.at(id);
    }
  }

  if (nValidElements == 0) {
    ROS_DEBUG_STREAM("No valid elements for this area!");
    return false;
  }

  segmentId = std::max_element(segmentSizes.begin(), segmentSizes.end(), PlaneSegmentation::compareSegmentSize)->first;
  nBiggestElements = segmentSizes[segmentId];
  ROS_DEBUG_STREAM("Found " << nValidElements << " valid elements.");
  ROS_DEBUG_STREAM("Biggest segment with id " << segmentId << " has " << nBiggestElements << " elements.");
  return true;
}

bool PlaneSegmentation::compareSegmentSize(const std::pair<size_t, size_t>& segment1, const std::pair<size_t, size_t>& segment2)
{
  return segment1.second < segment2.second;
}

bool PlaneSegmentation::extendSegment(std::vector<Eigen::Array2i>& segmentIndeces, const size_t& id, Plane& plane)
{
  bool hasSegmentChanged = false;
  for (vector<Eigen::Array2i>::iterator i = unsegmentedIndeces_.begin(); i != unsegmentedIndeces_.end();) {
    if (belongsToSegment(*i, plane)) {
      averageWithPlane(*i, plane);
      segmentIndeces.push_back(*i);
      i = unsegmentedIndeces_.erase(i);
      hasSegmentChanged = true;
      publishForVisualization(segmentIndeces, id);
    } else {
      ++i;
    }
  }
  return hasSegmentChanged;
}

bool PlaneSegmentation::belongsToSegment(const Eigen::Array2i& index, Plane& referencePlane)
{
  Eigen::Vector3d position, normal;
  map_->getPosition3d("elevation", index, position);
  map_->getVector("surface_normal_", index, normal);

  if (computeDistanceToPlane(position, referencePlane) > planeDistanceTolerance_) return false;
  if (computeAngleBetweenVectors(normal, referencePlane.getNormal()) >  planeAngleTolerance_) return false;
  return true;
}

double PlaneSegmentation::computeDistanceToPlane(const Eigen::Vector3d& point, Plane& plane)
{
  Eigen::Vector3d offset = point - plane.getPosition();
  double cosAlpha = offset.dot(plane.getNormal()) / (plane.getNormal().norm() * offset.norm());
  double angleToPlane = fabs(acos(cosAlpha) - M_PI_2);
  double distanceToPlance = sin(angleToPlane) * offset.norm();
  return distanceToPlance;
}

double PlaneSegmentation::computeAngleBetweenVectors(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2)
{
  return acos(fabs(vector1.dot(vector2) / (vector1.norm() * vector2.norm())));
}

void PlaneSegmentation::averageWithPlane(const Eigen::Array2i& index, Plane& plane)
{
  Eigen::Vector3d position, normal;
  map_->getPosition3d("elevation", index, position);
  map_->getVector("surface_normal_", index, normal);
  // TODO Add weights depending on variance.
  plane.averageWith(position, normal);
}

void PlaneSegmentation::publishForVisualization()
{
  if (visualizationPublisher_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 pointCloud;
    map_->toPointCloud(pointCloud, "elevation");
    visualizationPublisher_.publish(pointCloud);
  }
}

void PlaneSegmentation::publishForVisualization(const std::vector<Eigen::Array2i>& indices, const size_t& segmentId)
{
  if (detailedVisualizationPublisher_.getNumSubscribers() > 0) {
    grid_map::GridMap mapCopy = *map_;
    for (const auto& i : indices)
    mapCopy.at(typeName_, i) = segmentId;
    sensor_msgs::PointCloud2 pointCloud;
    mapCopy.toPointCloud(pointCloud, "elevation");
    detailedVisualizationPublisher_.publish(pointCloud);
    sleepDurationBetweenSegmentationSteps_.sleep();
  }
}

} /* namespace foothold_finder */
