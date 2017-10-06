/*
 * PlaneSegmentation.hpp
 *
 *  Created on: Oct 16, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "foothold_finder/Plane.hpp"

// Grid Map
#include <grid_map/GridMap.hpp>
#include <grid_map_lib/iterators/PolygonIterator.hpp>

// Eigen
#include <Eigen/Core>

// ROS
#include <ros/ros.h>

// STD
#include <string>
#include <vector>
#include <map>

namespace foothold_finder {

/*!
 * Plane segmentation of an elevation map.
 */
class PlaneSegmentation
{
 public:
  /*!
   * Constructur.
   * @param nodeHandle the ROS node handle.
   */
  PlaneSegmentation(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~PlaneSegmentation();

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Compute the segmentation.
   * @param[in/out] map the map to be segmented.
   * @return true if successful, false if map was already contains segmentation information.
   */
  bool segment(grid_map::GridMap& map);

  /*!
   * Get the plane for an id from the segmentation process.
   * @param segmentId the segment id to get the plane for.
   * @return the plane parameters of the segment.
   */
  Plane& getPlane(const size_t& segmentId);

  /*!
   * Get the max. slope angle for the plane corresponding to an id from the segmentation process.
   * @param segmentId the segment id to get the plane for.
   * @return the slope angle.
   */
  double getSlopeAngle(const size_t& segmentId);

  /*!
   * Get the map which is segmented.
   * @return pointer to the map.
   */
  const grid_map::GridMap* getMap();

  /*!
   * Get the id of the biggest segment for an area.
   * @param[in] polygon the area to be checked.
   * @param[out] segmentId the segment id of the biggest segment.
   * @param[out] nBiggestElements the number of elements of the biggest segment.
   * @param[out] nValidElements the number of valid elements.
   * @param[out] nElements the number of elements in the area.
   * @return true if successful, false if no valid data available.
   */
  typedef grid_map_lib::PolygonIterator::Polygon Polygon;
  bool getBiggestSegmentForArea(const Polygon& polygon, size_t& segmentId, size_t& nBiggestElements, size_t& nValidElements, size_t& nElements);

  const std::string typeName_ = "segmentation_id";
  std::vector<std::string> typeNameList_;

 private:

  /*!
   * Initialize segmentation process.
   * @return true if successful.
   */
  bool initializeSegmentation();

  /*!
   * Compare two segments by their size.
   */
  static bool compareSegmentSize(const std::pair<size_t, size_t>& segment1, const std::pair<size_t, size_t>& segment2);

  /*!
   * Extend segment by checking if unsegmented parts belong to the same segment.
   * @param[in/out] segmentIndeces the existing indeces of the segment to be extended.
   * @param[in] id the id of the segment.
   * @param[in/out] plane the average plane of the segment.
   * @return true if segment has changed, false otherwise.
   */
  bool extendSegment(std::vector<Eigen::Array2i>& segmentIndeces, const size_t& id, Plane& plane);

  /*!
   * Checks if a cell of the map belongs to a given segment (belongs to the same plane).
   * @param index the index of the cell to check.
   * @param referencePlane the plane of the cluster.
   * @return true if same cluster, false if not.
   */
  bool belongsToSegment(const Eigen::Array2i& index, Plane& referencePlane);

  /*!
   * Computes the distance of a point to a plane.
   * @param offset the position of the point.
   * @param plane the to which the distance is computed.
   * @return distance of the point to the plane.
   */
  double computeDistanceToPlane(const Eigen::Vector3d& point, Plane& plane);

  /*!
   * Computes the angle between two vectors.
   * @param vector1
   * @param vector2
   * @return angle in rad.
   */
  double computeAngleBetweenVectors(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2);

  /*!
   * Adds the data of a cell to the plane average.
   * @param index the cell to be added.
   * @param planeAverage the plane average to be updated.
   */
  void averageWithPlane(const Eigen::Array2i& index, Plane& plane);

  /*!
   * Publish segmented map as point cloud for visualization/debugging.
   */
  void publishForVisualization();

  /*!
   * Add a list of indices (temporary) and publish segmented map as point cloud
   * for visualization/debugging.
   * @param indices the indices to be added to a copy of the map.
   * @param segmentId the segmentation id of the indices.
   */
  void publishForVisualization(const std::vector<Eigen::Array2i>& indices, const size_t& segmentId);

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Map as point cloud publisher (for visualization purposes).
  ros::Publisher visualizationPublisher_;

  //! Map as point cloud publisher (for visualization purposes) for step by step segmentation.
  ros::Publisher detailedVisualizationPublisher_;

  //! Map to be segmented.
  grid_map::GridMap* map_;

  //! Planes of the map found from segmentation.
  std::map<unsigned int, Plane> planes_;

  //! All valid indices to be segmented.
  std::vector<Eigen::Array2i> unsegmentedIndeces_;

  //! Tolerance on how close the points have to lie on a plane to be regarded as same plane.
  double planeDistanceTolerance_;

  //! Tolerance on how close the surface normals have to match to be regarded as same plane.
  double planeAngleTolerance_;

  //! Minimum number of cells per segment.
  int minCellsForSegment_;

  //! Maximum number of steps for the optimization.
  int maxOptimizationSteps_;

  //! Duration to wait/sleep when publishing single segmentation steps (for debugging) [s].
  ros::Duration sleepDurationBetweenSegmentationSteps_;
};

} /* namespace foothold_finder */
