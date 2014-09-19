/*
 * LidarToPointCloudTransformer.hpp
 *
 *  Created on: Aug 21, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "lidar_to_point_cloud_transformer/PointCloudAssembler.hpp"

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <filters/filter_chain.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
// PCL
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <string>
#include <limits>


using namespace point_containment_filter;

class RobotSelfFilter
{
	  mutable boost::recursive_mutex shape_handles_lock_;
public:
	 typedef std::map<const robot_model::LinkModel*, std::vector<std::pair<ShapeHandle, std::size_t> > > LinkShapeHandles;
	 typedef std::map<ShapeHandle, Eigen::Affine3d, std::less<ShapeHandle>,
	                  Eigen::aligned_allocator<std::pair<const ShapeHandle, Eigen::Affine3d> > > ShapeTransformCache;

	LinkShapeHandles link_shape_handles_;
	boost::scoped_ptr<ShapeMask> shape_mask_;
	boost::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader;
	ShapeTransformCache transformCache;
	const double scale=1;
	const double padding=0.1;
	const double max_range_=std::numeric_limits<double>::max();
	tf::TransformListener tfListener;

	RobotSelfFilter();

	void addAllLinksToShapeMask();

	bool getShapeTransform(ShapeHandle h, Eigen::Affine3d &transform) const;

	~RobotSelfFilter(){};

//	inline void filterPointClould(sensor_msgs::PointCloud2::Ptr & cloud_msg)
//	{
//		ROS_ERROR("not implemented, supporting moveit-current (09/07/2014)");
//	};

	void filterPointClould(const pcl::PointCloud<pcl::PointXYZI> & source_cloud, pcl::PointCloud<pcl::PointXYZI>& filtered_cloud);
	bool updateShapeTransformCache(const std::string &target_frame, const ros::Time &target_time);
};

namespace lidar_to_point_cloud_transformer {

/*!
 * Transforms the data from a rotating lidar to a point cloud per scan and an assembled point cloud per rotation.
 */
class LidarToPointCloudTransformer
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  LidarToPointCloudTransformer(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~LidarToPointCloudTransformer();

  /*!
   * Definition of the point cloud type.
   */
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
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& laserScan);

  /*!
   * Convert laser scan to point cloud.
   * @param[in] laserScan the laser scan to be converted.
   * @param[out] pointCloud the resulting point cloud.
   * @return true if successful.
   */
  bool convertLaserScanToPointCloud(const sensor_msgs::LaserScan& laserScan, PointCloud::Ptr pointCloud);

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Point cloud assembler.
  PointCloudAssembler pointCloudAssembler_;

  //! TF listener.
  tf::TransformListener tfListener_;

  //! Subscriber to the laser scan.
  ros::Subscriber laserScanSubscriber_;

  //! Laser scan filter chain.
  filters::FilterChain<sensor_msgs::LaserScan> laserScanFilterChain_;

  //! Point cloud publisher for each scan.
  ros::Publisher pointCloudPublisher_;

  //! Point cloud selfFilter
  boost::shared_ptr<RobotSelfFilter> selfFilter;
  ros::Publisher filteredPointCloudPublisher_;

  //! Assembled point cloud publisher.
  ros::Publisher assembledPointCloudPublisher_;

  //! Topic name of the laser scan.
  std::string laserScanTopic_;

  //! Topic name of the laser scan.
  int laserScanSubscriptionQueueSize_;

  //! Frame id of the non-rotating base frame.
  std::string fixedFrameId_;

  //! Target frame of the resulting point cloud to publish.
  std::string targetFrameId_;

  //! Duration for TF lookup timeout.
  ros::Duration tfLookupTimeoutDuration_;

  //! Projection from laser to point cloud.
  laser_geometry::LaserProjection laserProjector_;


};

} /* namespace */
