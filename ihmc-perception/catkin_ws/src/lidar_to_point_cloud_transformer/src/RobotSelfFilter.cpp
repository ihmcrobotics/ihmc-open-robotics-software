/*
 * RobotSelfFilters.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: unknownid
 */

#include <lidar_to_point_cloud_transformer/RobotSelfFilter.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <exception>

RobotSelfFilter::RobotSelfFilter(std::string& robot_description_param, double padding) :
		RobotSelfFilter::RobotSelfFilter(
				robot_model_loader::RobotModelLoader(robot_description_param), padding) {
}

RobotSelfFilter::RobotSelfFilter(
		const robot_model_loader::RobotModelLoader& model, double padding) :
		robot_model_loader(model), padding(padding) {
	shape_mask_.reset(new ShapeMask());
	addAllLinksToShapeMask();
	shape_mask_->setTransformCallback(
			boost::bind(&RobotSelfFilter::getShapeTransform, this, _1, _2));
}

void RobotSelfFilter::addAllLinksToShapeMask() {
	const std::vector<const robot_model::LinkModel*> &links =
			robot_model_loader.getModel()->getLinkModelsWithCollisionGeometry();
//		robot_model_loader->getModel()->getLinkModels(); //no-collision links
	for (std::size_t i = 0; i < links.size(); ++i) {
		ROS_INFO_STREAM(
				"link " << i << " " << links[i]->getName() << " shapes " << links[i]->getShapes().size());
		std::vector<shapes::ShapeConstPtr> shapes = links[i]->getShapes(); // copy shared ptrs
		for (std::size_t j = 0; j < shapes.size(); ++j) {
			ROS_INFO_STREAM(
					"shape " << j << " " << shapes::shapeStringName(&*shapes[j]));
			// merge mesh vertices up to 0.1 mm apart
			if (shapes[j]->type == shapes::MESH) {
				shapes::Mesh *m = static_cast<shapes::Mesh*>(shapes[j]->clone());
				m->mergeVertices(1e-4);
				shapes[j].reset(m);
			}

			ShapeHandle h = shape_mask_->addShape(shapes[j], scale, padding);
			if (h)
			{
				link_shape_handles_[links[i]].push_back(std::make_pair(h, j));
			}
		}
	}
}
;

bool RobotSelfFilter::getShapeTransform(ShapeHandle h,
		Eigen::Affine3d &transform) const {
	ShapeTransformCache::const_iterator it = transformCache.find(h);
	if (it == transformCache.end()) {
		ROS_ERROR("Internal error. Shape filter handle %u not found", h);
		return false;
	}
	transform = it->second;

	return true;
}



/*
 * refer planning_scene_monitor::PlanningSceneMonitor::getShapeTransformCache
 */
bool RobotSelfFilter::updateShapeTransformCache(const std::string &target_frame, const ros::Time &target_time) {
        boost::recursive_mutex::scoped_lock _(shape_handles_lock_);
        bool ret=true;
        for (LinkShapeHandles::const_iterator it = link_shape_handles_.begin(); it != link_shape_handles_.end(); ++it) {
                try {
                        tf::StampedTransform tr;
                        tfListener.lookupTransform(target_frame, it->first->getName(), target_time, tr);
                        Eigen::Affine3d ttr;
                        tf::transformTFToEigen(tr, ttr);
                        for (std::size_t j = 0; j < it->second.size(); ++j)
                        transformCache[it->second[j].first] =
                        ttr * it->first->getCollisionOriginTransforms()[it->second[j].second];
                } catch (tf::TransformException& ex) {
                        ROS_ERROR_THROTTLE(1, "Transform error: %s", ex.what());
                        ret=false;
                }
        }
        return ret;
}

void RobotSelfFilter::filterPointClould(
		const pcl::PointCloud<pcl::PointXYZI> & source_cloud,
		pcl::PointCloud<pcl::PointXYZI>& filtered_cloud, pcl::PointCloud<pcl::PointXYZI>& removed_cloud) {

	//update transform cache
	std_msgs::Header header;
	pcl_conversions::fromPCL(source_cloud.header, header);
	if(header.frame_id == "")
		throw std::runtime_error("source cloud frame_id cannot be empty");
	updateShapeTransformCache(header.frame_id, header.stamp);

	std::vector<int> mask_;
	const Eigen::Vector3d dummy_sensor_origin;
	const double min_range_ = 0.0;

	//convert XYZI to XYZ
	sensor_msgs::PointCloud2 tmp;
	pcl::toROSMsg(source_cloud, tmp);

	//obtain in/out mask
	shape_mask_->maskContainment(tmp, dummy_sensor_origin, min_range_,
			max_range_, mask_);

	//insert selected ones
	pcl::PointCloud<pcl::PointXYZI>::const_iterator cloud_it =
			source_cloud.begin();
	for (std::vector<int>::iterator mask_it = mask_.begin();
			mask_it != mask_.end(); mask_it++) {
		if (*mask_it == ShapeMask::OUTSIDE)
			filtered_cloud.push_back(*cloud_it);
		else
			removed_cloud.push_back(*cloud_it);
		cloud_it++;
	}

	filtered_cloud.header=source_cloud.header;
	removed_cloud.header=source_cloud.header;
}
