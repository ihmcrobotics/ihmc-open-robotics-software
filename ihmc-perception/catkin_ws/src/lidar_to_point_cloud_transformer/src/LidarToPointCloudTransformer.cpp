/*
 * LidarToPointCloudTransformer.cpp
 *
 *  Created on: Aug 21, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "lidar_to_point_cloud_transformer/LidarToPointCloudTransformer.hpp"

// ROS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

// PCL
#include <pcl_ros/transforms.h>

using namespace std;
using namespace point_containment_filter;
using namespace ros;
#include <sensor_msgs/LaserScan.h>

namespace lidar_to_point_cloud_transformer {

LidarToPointCloudTransformer::LidarToPointCloudTransformer(
		ros::NodeHandle& nodeHandle) :
		nodeHandle_(nodeHandle), pointCloudAssembler_(tfListener_), laserScanFilterChain_(
				"sensor_msgs::LaserScan")  {
	readParameters();
	laserScanSubscriber_ = nodeHandle_.subscribe(laserScanTopic_,
			laserScanSubscriptionQueueSize_,
			&LidarToPointCloudTransformer::laserScanCallback, this);
	pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(
			"lidar_point_cloud", 1);
	filteredKeptPointCloudPublisher_ =
			nodeHandle.advertise<sensor_msgs::PointCloud2>(
					"kept_point_cloud", 1);
	filteredRemovedPointCloudPublisher_ =
			nodeHandle.advertise<sensor_msgs::PointCloud2>(
					"removed_point_cloud", 1);
	assembledPointCloudPublisher_ = nodeHandle_.advertise<
			sensor_msgs::PointCloud2>("assembled_lidar_point_cloud", padding);
	selfFilter.reset(new RobotSelfFilter(robotDescription, padding));
	initialize();
}

bool LidarToPointCloudTransformer::readParameters() {
	nodeHandle_.param("laser_scan_topic", laserScanTopic_, string("scan"));
	nodeHandle_.param("laser_scan_subscription_queue_size",
			laserScanSubscriptionQueueSize_, 10);
	nodeHandle_.param("fixed_frame", fixedFrameId_, string("map"));
	nodeHandle_.param("target_frame", targetFrameId_, string("scanner"));
	nodeHandle_.param("filter_robot_description_topic", robotDescription, string("filter_robot_description"));
	nodeHandle_.param("filter_padding", padding, 0.0);

	double tfLookupTimeoutSeconds;
	nodeHandle_.param("tf_lookup_timeout_duration", tfLookupTimeoutSeconds,
			0.2);
	tfLookupTimeoutDuration_.fromSec(tfLookupTimeoutSeconds);

	string baseFrameId, rotatingFrameId;
	nodeHandle_.param("base_frame_id", baseFrameId, string("laser_base"));
	nodeHandle_.param("rotating_frame_id", rotatingFrameId, string("laser"));
	double limitAngle;
	nodeHandle_.param("limit_angle", limitAngle, M_PI);
	return pointCloudAssembler_.setParameters(limitAngle, baseFrameId,
			rotatingFrameId, tfLookupTimeoutDuration_);
}

bool LidarToPointCloudTransformer::initialize() {
	laserScanFilterChain_.configure("laser_scan_filter_chain", nodeHandle_);

	Duration(0.2).sleep(); // Need this for clock.
	while (!pointCloudAssembler_.acquireReferenceTransformation(Time::now())) {
		ROS_INFO("Waiting to get transform for rotating laser frame.");
		Duration(0.2).sleep();
	}

	ROS_INFO("rotating_lidar_to_pointcloud_transformer initialized.");

	return true;
}

void LidarToPointCloudTransformer::laserScanCallback(
		const sensor_msgs::LaserScanConstPtr& laserScan) {
	if (pointCloudPublisher_.getNumSubscribers() < 1
			&& assembledPointCloudPublisher_.getNumSubscribers() < 1
			&& filteredKeptPointCloudPublisher_.getNumSubscribers() < 1
			&& filteredRemovedPointCloudPublisher_.getNumSubscribers() < 1)
		return;

	const ros::Time& scantime = laserScan->header.stamp;

	// Filter laser scan.
	sensor_msgs::LaserScan laserScanFiltered;
	laserScanFilterChain_.update(*laserScan, laserScanFiltered);

	// Convert laser scan to point cloud.
	PointCloud::Ptr pointCloud(new PointCloud);
	if (!convertLaserScanToPointCloud(laserScanFiltered, pointCloud))
		return;

	// Transform point cloud to target frame.
	PointCloud::Ptr pointCloudTransformed(new PointCloud);
	if (!pcl_ros::transformPointCloud(targetFrameId_, *pointCloud,
			*pointCloudTransformed, tfListener_))
		return;

	// Robot self filter.
	pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudFilteredKept(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudFilteredRemoved(new pcl::PointCloud<pcl::PointXYZI>());
	selfFilter->filterPointClould(*pointCloud, *pointCloudFilteredKept, *pointCloudFilteredRemoved);
	filteredKeptPointCloudPublisher_.publish(pointCloudFilteredKept);
	filteredRemovedPointCloudPublisher_.publish(pointCloudFilteredRemoved);

	// Publish laser scan as point cloud.
	pointCloudPublisher_.publish(pointCloudTransformed);

	// Assemble point cloud.
	PointCloud::Ptr pointCloudAssembled(new PointCloud);
	bool assemblyCycleFinished;
	if (!pointCloudAssembler_.assemble(pointCloudFilteredKept, pointCloudAssembled,
			assemblyCycleFinished))
		return;

	// Publish assembled point cloud.
	if (assemblyCycleFinished) {
		if (assembledPointCloudPublisher_.getNumSubscribers() < 1)
			return;
		PointCloud::Ptr pointCloudAssembledTransformed(new PointCloud);
		if (!pcl_ros::transformPointCloud(targetFrameId_, *pointCloudAssembled,
				*pointCloudAssembledTransformed, tfListener_))
			return;
		assembledPointCloudPublisher_.publish(pointCloudAssembledTransformed);
	}
}

bool LidarToPointCloudTransformer::convertLaserScanToPointCloud(
		const sensor_msgs::LaserScan& laserScan, PointCloud::Ptr pointCloud) {
	Time scanEndTime = laserScan.header.stamp
			+ Duration().fromSec((double) laserScan.scan_time);

	int mask = laser_geometry::channel_option::Intensity
			+ laser_geometry::channel_option::Distance
			+ laser_geometry::channel_option::Index
			+ laser_geometry::channel_option::Timestamp;
	sensor_msgs::PointCloud2 pointCloudMessage;

	if (!tfListener_.waitForTransform(fixedFrameId_, laserScan.header.frame_id,
			scanEndTime, tfLookupTimeoutDuration_)) {
		ROS_ERROR("Failed to lookup transform from %s to %s for time %f.",
				laserScan.header.frame_id.c_str(), fixedFrameId_.c_str(),
				scanEndTime.toSec());
		return false;
	}
	try {
		laserProjector_.transformLaserScanToPointCloud(fixedFrameId_, laserScan,
				pointCloudMessage, tfListener_, -1.0, mask);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
		return false;
	}

	// Convert to PCL point cloud.
	pcl::PCLPointCloud2 pclPointCloud;
	pcl_conversions::toPCL(pointCloudMessage, pclPointCloud);
	for (auto& field : pclPointCloud.fields) {
		if (field.name == "intensities")
			field.name = "intensity";
	}

	pcl::fromPCLPointCloud2(pclPointCloud, *pointCloud);
	return true;
}

} /* namespace */
