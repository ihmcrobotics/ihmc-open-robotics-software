/*
 * FootholdFinder.hpp
 *
 *  Created on: Sep 17, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

// Grid map
#include <grid_map_msg/GridMap.h>
#include <grid_map/GridMap.hpp>
#include <grid_map_lib/PolygonIterator.hpp>

// Foothold finder
#include <foothold_finding_msg/AdaptFootholds.h>

// ROS
#include <ros/ros.h>

// Kindr
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>
#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/poses/PoseEigen.hpp>

// STD
#include <vector>
#include <string>

namespace foothold_finder {

/*!
 * Foot hold search and optimization.
 */
class FootholdFinder
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  FootholdFinder(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~FootholdFinder();

  typedef kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD Pose;
  typedef kindr::rotations::eigen_impl::RotationQuaternionPD Rotation;
  typedef grid_map_lib::PolygonIterator::Polygon Polygon;

 private:

  const bool isDebug_ = true;

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initialization.
   * @return true if successful.
   */
  bool initialize();

  /*!
   * Callback for the foothold adaptation service.
   * @param request the service request.
   * @param response the service response.
   * @return true if successful.
   */
  bool adaptCallback(foothold_finding_msg::AdaptFootholds::Request& request, foothold_finding_msg::AdaptFootholds::Response& response);

  bool getMap(const Eigen::Vector2d& position, const double sideLength, grid_map::GridMap& map);

  void clusterPolygonAreaInMap(const Polygon& polygon, grid_map::GridMap& map);

  /*!
   * Request a part of the elevation map and cluster it.
   * @param[in] position the desired center of the submap.
   * @param[in] radius the search radius.
   * @param[out] map the clustered map.
   * @return true if successful, false if position is outside the map.
   */
  void clusterMap(const Eigen::Vector2d& position, const double radius, grid_map::GridMap& map);

  /*!
   * Positions and projects a rotated standard foot shape to the world horizontal plane.
   * @param[in] pose the pose of the foot.
   * @param[out] footShape the positioned and projected foot shape.
   */
  void getFootShapeInWorldFrame(const Pose& pose, Polygon& footShape);

  /*!
   * Checks is two cells belong to the same cluster (belong to the same plane).
   * @param map the map holding the data.
   * @param firstIndex the index of the reference cell to check.
   * @param secondIndex the index of the cell to check.
   * @return true if same cluster, false if not.
   */
  bool isSameCluster(const grid_map::GridMap& map, const Eigen::Array2i& firstIndex, const Eigen::Array2i& secondIndex);

  /*!
   * Computes the distance of a point to the plane specified by a point and a normal vector.
   * @param normal the normal vector of the plane.
   * @param offset the vector from the point on the plane to the point the distance is measured.
   * @return distance of the point to the plane.
   */
  double computeDistanceToNormalPlance(const Eigen::Vector3d& normal, const Eigen::Vector3d& offset);

  bool findCandidates(const foothold_finding_msg::Foothold& foothold, grid_map::GridMap& map,
                      std::vector<foothold_finding_msg::Foothold>& candidates);

  void checkAndAdaptFoothold(grid_map::GridMap& map, foothold_finding_msg::Foothold& foothold);

  int checkFoothold(const grid_map::GridMap& map, const foothold_finding_msg::Foothold& foothold);

  void matchToSurfaceNormal(const Eigen::Vector3d& normal, Rotation& rotation);

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Foothold adaptation service server.
  ros::ServiceServer footholdAdaptService_;

  //! Elevation map service client.
  ros::ServiceClient elevationMapClient_;

  //! Name of the elevation map service.
  std::string elevationMapServiceName_;

  //! Side length of the area to look for for each foothold.
  double searchAreaSideLength_;

  //! Tolerance on how close the points have to lie on a plane to be regarded as same plane.
  double planeTolerance_;

  //! Shape of the footprint in the foot frame;
  Polygon footShape_;

  //! Types to be requested from elevation map.
  std::vector<std::string> mapRequestTypes_;

  //! Map as point cloud publisher (for visualization purposes).
  ros::Publisher segmentationPublisher_;

//
//  //! ROS subscriber to the grid map.
//  ros::Subscriber mapSubscriber_;
//
//  //! Topic name of the grid map to be visualized.
//  std::string mapTopic_;
//
//  //! Map region visualization.
//  MapRegionVisualization mapRegionVisualization_;
//
//  //! Visualizing map as point cloud.
//  PointCloudVisualization pointCloudVisualization_;
//
//  //! Visualizing data as vectors.
//  VectorVisualization vectorVisualization_;
};

} /* namespace */
