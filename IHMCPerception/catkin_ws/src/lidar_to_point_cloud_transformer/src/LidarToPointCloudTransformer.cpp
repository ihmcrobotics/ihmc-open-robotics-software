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
#include <geometric_shapes/shape_operations.h>
#include <tf_conversions/tf_eigen.h>




// PCL
#include <pcl_ros/transforms.h>

using namespace std;
using namespace point_containment_filter;
using namespace ros;
#include <sensor_msgs/LaserScan.h>



RobotSelfFilter::RobotSelfFilter()
{
        robot_model_loader.reset(new robot_model_loader::RobotModelLoader("/Atlas/robot_description"));
        shape_mask_.reset(new ShapeMask());
        shape_mask_->setTransformCallback(boost::bind(&RobotSelfFilter::getShapeTransform, this, _1, _2));
        addAllLinksToShapeMask();

};

void RobotSelfFilter::addAllLinksToShapeMask()
{
        const std::vector<const robot_model::LinkModel*> &links = robot_model_loader->getModel()->getLinkModelsWithCollisionGeometry();
//		robot_model_loader->getModel()->getLinkModels(); //no-collision links
          for (std::size_t i = 0 ; i < links.size() ; ++i)
          {
            ROS_INFO_STREAM("link " << i << " " <<  links[i]->getName() << " shapes " << links[i]->getShapes().size());
            std::vector<shapes::ShapeConstPtr> shapes = links[i]->getShapes(); // copy shared ptrs
            for (std::size_t j = 0 ; j < shapes.size() ; ++j)
            {
              ROS_INFO_STREAM("shape " << j << " " << shapes::shapeStringName(&*shapes[j]) );
              // merge mesh vertices up to 0.1 mm apart
              if (shapes[j]->type == shapes::MESH)
              {
                shapes::Mesh *m = static_cast<shapes::Mesh*>(shapes[j]->clone());
                m->mergeVertices(1e-4);
                shapes[j].reset(m);
              }

              ShapeHandle h=shape_mask_->addShape(shapes[j], scale, padding);
              if (h)
                link_shape_handles_[links[i]].push_back(std::make_pair(h, j));
            }
          }
};

bool RobotSelfFilter::getShapeTransform(ShapeHandle h, Eigen::Affine3d &transform) const
{
  ShapeTransformCache::const_iterator it = transformCache.find(h);
  if (it == transformCache.end())
  {
    ROS_ERROR("Internal error. Shape filter handle %u not found", h);
    return false;
  }
  transform = it->second;

  return true;
};


void RobotSelfFilter::filterPointClould(const pcl::PointCloud<pcl::PointXYZI> & source_cloud, pcl::PointCloud<pcl::PointXYZI>& filtered_cloud)
{
        std::vector<int> mask_;
        const Eigen::Vector3d dummy_sensor_origin;
        const double min_range_=0.0;

        //convert XYZI to XYZ
        pcl::PointCloud<pcl::PointXYZ> tmp;
        pcl::copyPointCloud(source_cloud, tmp);

        //obtain in/out mask
        shape_mask_->maskContainment(tmp, dummy_sensor_origin, min_range_, max_range_, mask_);

        //insert selected ones
        pcl::PointCloud<pcl::PointXYZI>::const_iterator cloud_it = source_cloud.begin();
        for(vector<int>::iterator mask_it=mask_.begin(); mask_it!=mask_.end(); mask_it++)
        {
        	if(*mask_it==ShapeMask::OUTSIDE)
        		filtered_cloud.push_back(*cloud_it);
        	cloud_it++;
        }

        if(filtered_cloud.header.frame_id.empty())
        	filtered_cloud.header.frame_id = source_cloud.header.frame_id;

        if(filtered_cloud.header.frame_id != source_cloud.header.frame_id)
        	ROS_ERROR("filterPointCloud: source_cloud/filtered_cloud frame_id mismatch");
};



/*
 * refer planning_scene_monitor::PlanningSceneMonitor::getShapeTransformCache
 */
bool RobotSelfFilter::updateShapeTransformCache(const std::string &target_frame, const ros::Time &target_time)
{
  try
  {
          boost::recursive_mutex::scoped_lock _(shape_handles_lock_);
        for (LinkShapeHandles::const_iterator it = link_shape_handles_.begin() ; it != link_shape_handles_.end(); ++it)
        {
          tf::StampedTransform tr;
          tfListener.lookupTransform(target_frame, it->first->getName(), target_time, tr);
          Eigen::Affine3d ttr;
          tf::transformTFToEigen(tr, ttr);
          for (std::size_t j = 0 ; j < it->second.size() ; ++j)
                transformCache[it->second[j].first] = ttr * it->first->getCollisionOriginTransforms()[it->second[j].second];
        }
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_THROTTLE(1, "Transform error: %s", ex.what());
    return false;
  }
  return true;
};


namespace lidar_to_point_cloud_transformer {



LidarToPointCloudTransformer::LidarToPointCloudTransformer(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),

      pointCloudAssembler_(tfListener_),
      laserScanFilterChain_("sensor_msgs::LaserScan")
{
  readParameters();
  laserScanSubscriber_ = nodeHandle_.subscribe(laserScanTopic_, laserScanSubscriptionQueueSize_, &LidarToPointCloudTransformer::laserScanCallback, this);
  pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("lidar_point_cloud", 1);
  filteredPointCloudPublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud",1);
  assembledPointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("assembled_lidar_point_cloud", 1);
  initialize();
  selfFilter.reset(new RobotSelfFilter());
}



LidarToPointCloudTransformer::~LidarToPointCloudTransformer() {

}

bool LidarToPointCloudTransformer::readParameters() {
  nodeHandle_.param("laser_scan_topic", laserScanTopic_, string("scan"));
  nodeHandle_.param("laser_scan_subscription_queue_size", laserScanSubscriptionQueueSize_, 10);
  nodeHandle_.param("fixed_frame", fixedFrameId_, string("map"));
  nodeHandle_.param("target_frame", targetFrameId_, string("scanner"));

  double tfLookupTimeoutSeconds;
  nodeHandle_.param("tf_lookup_timeout_duration", tfLookupTimeoutSeconds, 0.2);
  tfLookupTimeoutDuration_.fromSec(tfLookupTimeoutSeconds);

  string baseFrameId, rotatingFrameId;
  nodeHandle_.param("base_frame_id", baseFrameId, string("laser_base"));
  nodeHandle_.param("rotating_frame_id", rotatingFrameId, string("laser"));
  double limitAngle;
  nodeHandle_.param("limit_angle", limitAngle, M_PI);
  return pointCloudAssembler_.setParameters(limitAngle, baseFrameId, rotatingFrameId, tfLookupTimeoutDuration_);
}

bool LidarToPointCloudTransformer::initialize() {
  laserScanFilterChain_.configure("laser_scan_filter_chain", nodeHandle_);

  Duration(0.2).sleep(); // Need this for clock.
  while (!pointCloudAssembler_.acquireReferenceTransformation(Time::now()))
  {
    ROS_INFO("Waiting to get transform for rotating laser frame.");
    Duration(0.2).sleep();
  }

  ROS_INFO("rotating_lidar_to_pointcloud_transformer initialized.");

  return true;
}

void LidarToPointCloudTransformer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& laserScan) {
  if (pointCloudPublisher_.getNumSubscribers() < 1 && assembledPointCloudPublisher_.getNumSubscribers() < 1) return;

  const ros::Time& scantime = laserScan->header.stamp;

  // Filter laser scan.
  sensor_msgs::LaserScan laserScanFiltered;
  laserScanFilterChain_.update(*laserScan, laserScanFiltered);

  // Convert laser scan to point cloud.
  PointCloud::Ptr pointCloud(new PointCloud);
  if (!convertLaserScanToPointCloud(laserScanFiltered, pointCloud)) return;


  // Transform point cloud to target frame.
  PointCloud::Ptr pointCloudTransformed(new PointCloud);
  if (!pcl_ros::transformPointCloud(targetFrameId_, *pointCloud, *pointCloudTransformed, tfListener_)) return;

  // Robot self filter.
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudFiltered(new pcl::PointCloud<pcl::PointXYZI>());
  selfFilter->updateShapeTransformCache(pointCloud->header.frame_id, scantime);
  selfFilter->filterPointClould(*pointCloud, *pointCloudFiltered);
  filteredPointCloudPublisher_.publish(pointCloudFiltered);

  // Publish laser scan as point cloud.
  pointCloudPublisher_.publish(pointCloudTransformed);

  // Assemble point cloud.
  PointCloud::Ptr pointCloudAssembled(new PointCloud);
  bool assemblyCycleFinished;
  if (!pointCloudAssembler_.assemble(pointCloudFiltered, pointCloudAssembled, assemblyCycleFinished)) return;

  // Publish assembled point cloud.
  if (assemblyCycleFinished) {
    if (assembledPointCloudPublisher_.getNumSubscribers() < 1) return;
    PointCloud::Ptr pointCloudAssembledTransformed(new PointCloud);
    if (!pcl_ros::transformPointCloud(targetFrameId_, *pointCloudAssembled, *pointCloudAssembledTransformed, tfListener_)) return;
    assembledPointCloudPublisher_.publish(pointCloudAssembledTransformed);
  }
}

bool LidarToPointCloudTransformer::convertLaserScanToPointCloud(const sensor_msgs::LaserScan& laserScan, PointCloud::Ptr pointCloud) {
  Time scanEndTime =  laserScan.header.stamp + Duration().fromSec((double) laserScan.scan_time);

  int mask = laser_geometry::channel_option::Intensity +
             laser_geometry::channel_option::Distance +
             laser_geometry::channel_option::Index +
             laser_geometry::channel_option::Timestamp;
  sensor_msgs::PointCloud2 pointCloudMessage;

  if (!tfListener_.waitForTransform(fixedFrameId_, laserScan.header.frame_id, scanEndTime, tfLookupTimeoutDuration_)) {
    ROS_ERROR("Failed to lookup transform from %s to %s for time %f.", laserScan.header.frame_id.c_str(), fixedFrameId_.c_str(), scanEndTime.toSec());
    return false;
  }
  try {
    laserProjector_.transformLaserScanToPointCloud(fixedFrameId_, laserScan, pointCloudMessage, tfListener_, -1.0, mask);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  // Convert to PCL point cloud.
  pcl::PCLPointCloud2 pclPointCloud;
  pcl_conversions::toPCL(pointCloudMessage, pclPointCloud);
  for (auto& field : pclPointCloud.fields) {
    if (field.name == "intensities") field.name = "intensity";
  }

  pcl::fromPCLPointCloud2(pclPointCloud, *pointCloud);
  return true;
}

} /* namespace */
