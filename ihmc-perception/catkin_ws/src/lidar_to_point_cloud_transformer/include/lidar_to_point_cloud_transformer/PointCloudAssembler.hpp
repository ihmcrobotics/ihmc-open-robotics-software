/*
 * PointCloudAssembler.hpp
 *
 *  Created on: Aug 21, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Kindr
#include <kindr/rotations/RotationEigen.hpp>

// PCL
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

namespace lidar_to_point_cloud_transformer {

/*!
 * Builds a point cloud from a full rotation of the lidar.
 */
class PointCloudAssembler
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  PointCloudAssembler(tf::TransformListener& tfListener);

  /*!
   * Destructor.
   */
  virtual ~PointCloudAssembler();

  /*!
   * Definition of the point cloud type.
   */
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

  /*!
   * Set parameters. Set parameters before using this class.
   * @param limitAngle the angle at which the point cloud should be assembled.
   * @param baseFrameId the frame id of the non-rotating base frame.
   * @param rotatingFrameId the frame id of the rotating frame.
   * @param tfLookupTimeoutDuration the duration for TF lookup timeout.
   * @return true if successful.
   */
  bool setParameters(const double limitAngle,
                     const std::string baseFrameId,const std::string rotatingFrameId,
                     ros::Duration tfLookupTimeoutDuration);

  /*!
   * Acquire a new reference transform from which the angle is counted from.
   * @param timestamp the time for which the reference transform should be acquired.
   * @return true if successful.
   */
  bool acquireReferenceTransformation(const ros::Time& timestamp);

  /*!
   * Add a single point cloud to be assembled. Returns the assembled point cloud
   * if the assembly cycle is finished.
   * @param[in] pointCloud the point cloud to add to the assembled point cloud.
   * @param[out] pointCloudAssembled the assembled point cloud (if cycle is finished, empty otherwise).
   * @param[out] assemblyCycleFinished true if assembly cycle is finished, false if it is in progress.
   * @return true if successful, false otherwise.
   */
  bool assemble(PointCloud::ConstPtr pointCloud, PointCloud::Ptr pointCloudAssembled, bool& assemblyCycleFinished);

 private:
  /*!
   * Gets the transform for the rotation of the laser.
   * @param[in] timestamp the time for which the rotation is get.
   * @param[out] transform the resulting transform.
   * @return true if successful.
   */
  bool getLaserRotationTransform(const ros::Time& timestamp, tf::StampedTransform& transform);

  /*!
   * Checks if the current transformation is over the defined limit.
   * @param transformation the current transform to check for.
   * @return true if limit is reached, false otherwise.
   */
  bool checkIfTransformationOverLimit(const tf::Transform& transformation);

  //! True if parameters are set.
  bool isParametersSet_;

  //! Assembled point cloud.
  PointCloud::Ptr pointCloudAssembled_;

  //! TF listener.
  tf::TransformListener& tfListener_;

  //! Duration for TF lookup timeout.
  ros::Duration tfLookupTimeoutDuration_;

  //! Frame id of the non-rotating base frame.
  std::string baseFrameId_;

  //! Frame id of the rotating frame.
  std::string rotatingFrameId_;

  //! Last rotation of the rotating frame.
  kindr::rotations::eigen_impl::AngleAxisPD lastRotation_;

  //! Current angle of the rotating frame.
  double angle_;

  //! Angle at which the point cloud should be assembled.
  double limitAngle_;
};

} /* namespace */
