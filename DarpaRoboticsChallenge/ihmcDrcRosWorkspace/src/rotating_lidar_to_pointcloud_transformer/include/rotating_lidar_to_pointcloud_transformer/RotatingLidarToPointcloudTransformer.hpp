/*
 * RotatingLidarToPointcloudTransformer.hpp
 *
 *  Created on: Aug 21, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_assembler/AssembleScans2.h>

// Kindr
#include <kindr/rotations/RotationEigen.hpp>

// PCL
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace rotating_lidar_to_pointcloud_transformer {

/*!
 * Subscribes to a laserscan of a rotating laser and builds a point cloud from a full rotation.
 */
class RotatingLidarToPointcloudTransformer
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  RotatingLidarToPointcloudTransformer(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RotatingLidarToPointcloudTransformer();

  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

 private:

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Performs the initialization procedure.
   * @return true if successful.
   */
  bool initialize();

  /*!
   * Callback function for the laser spin angle check timer.
   * @param timerEvent the timer event.
   */
  void timerCallback(const ros::TimerEvent& timerEvent);

  /*!
   * Tries to acquire a new reference transform from which the angle is counted from.
   * @param timestamp the time for which the reference transform should be acquired.
   * @return true if successful.
   */
  bool acquireReferenceTransformation(const ros::Time& timestamp);

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

  bool filterPointCloud(PointCloud::ConstPtr pointCloud, PointCloud::Ptr pointCloudFiltered);

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Service client for calling the assembler.
  ros::ServiceClient assemblerServiceClient_;

  //! Assembled point cloud publisher.
  ros::Publisher assembledPointCloudPublisher_;

  //! Timer to keep track of rotation of laser sensor.
  ros::Timer timer_;

  //! TF listener.
  tf::TransformListener tfListener_;

  //! Time of the last laser assembly.
  ros::Time timeOfLastAssembly_;

  //! Last rotation of the rotating frame.
  kindr::rotations::eigen_impl::AngleAxisPD lastRotation_;

  //! Current angle of the rotating frame.
  double angle_;

  //! Name of the laser assembler service.
  std::string laserAssemblerName_;

  //! Angle at which the point cloud should be assembled.
  double limitAngle_;

  //! Frame id of the non-rotating base frame.
  std::string baseFrameId_;

  //! Frame id of the rotating frame.
  std::string rotatingFrameId_;

  //! Frame of the resulting point cloud to export.
  std::string exportFrameId_;

  //! Duration of the timer to check the laser spin angle.
  ros::Duration timeDuration_;

  //! Duration for TF lookup timeout.
  ros::Duration tfLookupTimeoutDuration_;

  double intersectionFilterRadius_;


};

} /* namespace */
