/*
 * FootholdFinder.hpp
 *
 *  Created on: Sep 17, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

#include "foothold_finder/Plane.hpp"
#include "foothold_finder/PlaneSegmentation.hpp"

// Grid map
#include <grid_map_msg/GridMap.h>
#include <grid_map/GridMap.hpp>
#include <grid_map_lib/iterators/PolygonIterator.hpp>

// Foothold finder
#include <foothold_finding_msg/AdaptFootholds.h>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

  /*!
   * Request the map around position and given size.
   * @param[in] position the desired position of the map (center)
   * @param[in] sideLength the side lengths of the desired map.
   * @param[out] map the acquired map.
   * @return true if successful, false otherwise.
   */
  bool getMap(const Eigen::Vector2d& position, const double sideLength, grid_map::GridMap& map);

  /*!
   * Exploration and optimization of the foothold.
   * @param[in] foothold to optimize.
   * @param[in] segmentation the segmentation information containing the map and the planes.
   * @param[out] candidates the found candidates.
   * @return true if successful, false otherwise.
   */
  bool findCandidates(const foothold_finding_msg::Foothold& foothold, PlaneSegmentation& segmentation, std::vector<foothold_finding_msg::Foothold>& candidates);

  /*!
   * Locally optimizes foothold with a search.
   * @param[in] segmentation the segmentation information containing the map and the planes.
   * @param[in/out] foothold the foothold to optimize.
   * @return true if candidate found, false otherwise.
   */
  bool localSearch(PlaneSegmentation& segmentation, foothold_finding_msg::Foothold& foothold);

  /*!
   * Checks if a foot hold is valid.
   * @param[in] segmentation the segmentation information containing the map and the planes.
   * @param[in/out] foothold the foothold to be checked.
   */
  void checkFoothold(PlaneSegmentation& segmentation, foothold_finding_msg::Foothold& foothold);

  /*!
   * Adapt a foothold to a plane in pitch and roll, but keep yaw orientation.
   * @param[in] plane the plane to adapt the foothold to.
   * @param[out] foothold the foothold to adapt.
   */
  void adapt(Plane& plane, foothold_finding_msg::Foothold& foothold);

  /*!
   * Positions and projects a polygon to the map horizontal plane based on a pose.
   * @param referencePolygon the input polygon.
   * @param pose the desired pose of the output polygon.
   * @param polygonTransformed the output polygon.
   */
  void getPolygonInMapFrame(const Polygon& referencePolygon, const Pose& pose, Polygon& polygonTransformed);

  /*!
   * Publish foot shape as marker for visualization.
   * @param[in] footShape n map frame the foot shape to be published.
   * @param[in] plane the plane of the foothold.
   */
  void publishFootShape(const Polygon& footShape, Plane& plane);

  /*!
   * Set the roll and pitch of a rotation to align the surface normal.
   * @param[in] normal the surface normal to be matched to.
   * @param[out] rotation the rotation to align.
   */
  void matchToSurfaceNormal(const Eigen::Vector3d& normal, Rotation& rotation);

  /*! Helper function to get a debug string of a rotation.
   * @param rotation the rotation to get the information form.
   * @return the debug string.
   */
  std::string getDebugForm(const Rotation& rotation);

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Foothold adaptation service server.
  ros::ServiceServer footholdAdaptService_;

  //! Elevation map service client.
  ros::ServiceClient elevationMapClient_;

  //! Name of the elevation map service.
  std::string elevationMapServiceName_;

  //! Name of the elevation map frame.
  std::string mapFrame_;

  //! Foot shape publisher for debugging.
  ros::Publisher footShapePublisher_;

  //! Segmentation method.
  PlaneSegmentation segmentation_;

  //! Side length of the area to look for for each foothold.
  double searchRegionSideLength_;

  //! Percentage of valid cells for a valid foothold.
  double validCellCountThreshold_;

  //! Maximum allowed slope angle for a valid foothold.
  double maxSlopeAngle_;

  //! Shape of the footprint in the foot frame;
  Polygon footShape_;

  //! Shape of the region to search for each foothold;
  Polygon searchRegion_;

  //! Marker for the foot shape to be published.
  visualization_msgs::Marker footShapeMarker_;

  //! Line width of the foot shape published polygon.
  double footShapeLineWidth_;

  //! Offset to be added to the visualization in z-direction.
  double footShapeVisualizationOffset_;

  //! Types to be requested from elevation map.
  std::vector<std::string> mapRequestTypes_;
};

} /* namespace */
