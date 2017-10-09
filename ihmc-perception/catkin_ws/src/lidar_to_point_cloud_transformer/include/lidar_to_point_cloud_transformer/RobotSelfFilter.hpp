/*
 * RobotSelfFilters.h
 *
 *  Created on: Sep 22, 2014
 *      Author: unknownid
 */

#ifndef ROBOTSELFFILTERS_H_
#define ROBOTSELFFILTERS_H_
#include <moveit/point_containment_filter/shape_mask.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf/transform_listener.h>
#include <geometric_shapes/shape_operations.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/io.h>

#include <boost/thread.hpp>
using namespace point_containment_filter;

class RobotSelfFilter
{
	mutable boost::recursive_mutex shape_handles_lock_;
	 typedef std::map<const robot_model::LinkModel*, std::vector<std::pair<ShapeHandle, std::size_t> > > LinkShapeHandles;
	 typedef std::map<ShapeHandle, Eigen::Affine3d, std::less<ShapeHandle>,
	                  Eigen::aligned_allocator<std::pair<const ShapeHandle, Eigen::Affine3d> > > ShapeTransformCache;
	LinkShapeHandles link_shape_handles_;
	boost::scoped_ptr<ShapeMask> shape_mask_;
	robot_model_loader::RobotModelLoader robot_model_loader;
	ShapeTransformCache transformCache;
	tf::TransformListener tfListener;
	void addAllLinksToShapeMask();
	bool getShapeTransform(ShapeHandle h, Eigen::Affine3d &transform) const;
	const double scale=1;
	const double max_range_=std::numeric_limits<double>::max();
	double padding=0.1;
public:


	RobotSelfFilter(std::string& robot_description_param, double padding);
	RobotSelfFilter(const robot_model_loader::RobotModelLoader& model, double padding);
	bool updateShapeTransformCache(const std::string &target_frame, const ros::Time &target_time);

	~RobotSelfFilter(){};

	void filterPointClould(const pcl::PointCloud<pcl::PointXYZI> & source_cloud, pcl::PointCloud<pcl::PointXYZI>& filtered_cloud,
				pcl::PointCloud<pcl::PointXYZI>& removed_cloud);
};

#endif /* ROBOTSELFFILTERS_H_ */
