#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "transform_provider/TransformProvider.h"

tf::TransformListener* transformListener;

bool getTransform(transform_provider::TransformProvider::Request &req, transform_provider::TransformProvider::Response &res)
{
	if(transformListener->waitForTransform(req.src, req.dest, req.time, ros::Duration(0.1)))
	{

	tf::StampedTransform dstTransform;
	transformListener->lookupTransform(req.src, req.dest, req.time, dstTransform);
	
	res.transform.header.stamp = dstTransform.stamp_;
	res.transform.header.frame_id = dstTransform.frame_id_;
	res.transform.child_frame_id = dstTransform.child_frame_id_;
	res.transform.transform.translation.x = dstTransform.getOrigin().getX();
	res.transform.transform.translation.y = dstTransform.getOrigin().getY();
	res.transform.transform.translation.z = dstTransform.getOrigin().getZ();
	res.transform.transform.rotation.x = dstTransform.getRotation().getX();
	res.transform.transform.rotation.y = dstTransform.getRotation().getY();
	res.transform.transform.rotation.z = dstTransform.getRotation().getZ();
	res.transform.transform.rotation.w = dstTransform.getRotation().getW();
	return true;
	}
	else
	{
		return false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "transform_provider");

	transformListener = new tf::TransformListener();
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("transform_provider", getTransform);

	ros::spin();
}	
