/*
 * PointCloudAssembler.cpp
 *
 *  Created on: Aug 21, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "lidar_to_point_cloud_transformer/PointCloudAssembler.hpp"

// Kindr
#include <kindr/poses/PoseEigen.hpp>
#include <kindr/thirdparty/ros/RosTfPoseEigen.hpp>

// PCL
#include <pcl_ros/transforms.h>
#include <pcl/common/io.h>

using namespace std;
using namespace ros;
using namespace kindr::poses::eigen_impl;
using namespace kindr::rotations::eigen_impl;

namespace lidar_to_point_cloud_transformer {

PointCloudAssembler::PointCloudAssembler(tf::TransformListener& tfListener)
    : tfListener_(tfListener),
      pointCloudAssembled_(new PointCloud) {
  isParametersSet_ = false;
  angle_ = 0.0;
}

PointCloudAssembler::~PointCloudAssembler() {

}

bool PointCloudAssembler::setParameters(const double limitAngle,
                                        const std::string baseFrameId, const std::string rotatingFrameId,
                                        ros::Duration tfLookupTimeoutDuration) {
  limitAngle_ = limitAngle;
  baseFrameId_ = baseFrameId;
  rotatingFrameId_ = rotatingFrameId;
  tfLookupTimeoutDuration_ = tfLookupTimeoutDuration;
  isParametersSet_ = true;
  return isParametersSet_;
}

bool PointCloudAssembler::assemble(PointCloud::ConstPtr pointCloud, PointCloud::Ptr pointCloudAssembled, bool& assemblyCycleFinished) {
  ROS_DEBUG_STREAM("Assembling point cloud.");

  if (!isParametersSet_) {
    ROS_ERROR("Parameters are not set for the PointCloudAssembler.");
    return false;
  }

  if (pointCloudAssembled_->empty()) {
    pcl::copyPointCloud(*pointCloud, *pointCloudAssembled_);
  } else {
    if (pointCloudAssembled_->header.frame_id != pointCloud->header.frame_id) {
      ROS_ERROR_STREAM("Point clouds cannot be concatenated as they have different frame ids: "
                      << pointCloudAssembled_->header.frame_id << ", " << pointCloud->header.frame_id << ".");
      return false;
    }
    *pointCloudAssembled_ += *pointCloud;
    pointCloudAssembled_->header.stamp = pointCloud->header.stamp;
  }

  Time timeStamp;
  timeStamp.fromSec(1e-6 * pointCloud->header.stamp); // Point cloud timestamp in microseconds!
  tf::StampedTransform currentStampedTransform;
  if (!getLaserRotationTransform(timeStamp, currentStampedTransform)) return false;
  assemblyCycleFinished = checkIfTransformationOverLimit(currentStampedTransform);

  if (assemblyCycleFinished) {
    ROS_DEBUG_STREAM("Assembling point cloud cycle finished.");
    pcl::copyPointCloud(*pointCloudAssembled_, *pointCloudAssembled);
    pointCloudAssembled_->clear();
  }

  return true;
}

bool PointCloudAssembler::acquireReferenceTransformation(const ros::Time& timestamp) {
  tf::StampedTransform transform;
  if (!getLaserRotationTransform(timestamp, transform)) {
    ROS_WARN("Could not acquire transform for laser rotation from %s to %s.", baseFrameId_.c_str(), rotatingFrameId_.c_str());
    return false;
  }
  HomogeneousTransformationPosition3RotationQuaternionD pose;
  convertFromRosTf(transform, pose);
  lastRotation_ = pose.getRotation();
  angle_ = 0;
  ROS_INFO("Acquired reference transformation at time %f.", timestamp.toSec());
  return true;
}

bool PointCloudAssembler::getLaserRotationTransform(const ros::Time& timestamp, tf::StampedTransform& transform) {
  ROS_DEBUG_STREAM("Getting laser rotation transform.");
  if (!tfListener_.waitForTransform(rotatingFrameId_, baseFrameId_, timestamp, tfLookupTimeoutDuration_)) {
    ROS_ERROR("Failed to lookup transform from %s to %s for time %f.", baseFrameId_.c_str(), rotatingFrameId_.c_str(), timestamp.toSec());
    return false;
  }

  try {
    tfListener_.lookupTransform(rotatingFrameId_, baseFrameId_, timestamp, transform);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  return true;
}

bool PointCloudAssembler::checkIfTransformationOverLimit(const tf::Transform& transformation) {
  kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD currentPose, lastPose;
  kindr::poses::eigen_impl::convertFromRosTf(transformation, currentPose);
  kindr::rotations::eigen_impl::AngleAxisPD currentRotation(currentPose.getRotation());

  angle_ += kindr::common::wrapTwoPI(currentRotation.getDisparityAngle(lastRotation_));
  lastRotation_ = currentRotation;

  ROS_DEBUG_STREAM("Current angle: " << angle_ << ", limit angle: " << limitAngle_ << ".");

  if (angle_ > limitAngle_) {
    angle_ = 0.0;
    return true;
  }
  return false;
}

} /* namespace */
