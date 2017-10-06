#include <fstream>

#include "ros/ros.h"

#include "pointmatcher/PointMatcher.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

#include "ethzasl_icp_mapper/MatchClouds.h"

#include "tf/tf.h"

using namespace std;

class CloudMatcher
{
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;

	ros::NodeHandle& n;
	
	PM::ICP icp;
	
	ros::ServiceServer service;
	
public:
	CloudMatcher(ros::NodeHandle& n);
	bool match(ethzasl_icp_mapper::MatchClouds::Request& req, ethzasl_icp_mapper::MatchClouds::Response& res);
};

CloudMatcher::CloudMatcher(ros::NodeHandle& n):
	n(n),
	service(n.advertiseService("match_clouds", &CloudMatcher::match, this))
{
	// load config
	string configFileName;
	if (ros::param::get("~config", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			icp.loadFromYaml(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load config from YAML file " << configFileName);
			icp.setDefault();
		}
	}
	else
	{
		ROS_WARN_STREAM("No config file specified, using default ICP chain.");
		icp.setDefault();
	}
	
	// replace logger
	if (getParam<bool>("useROSLogger", false))
		PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);
}

bool CloudMatcher::match(ethzasl_icp_mapper::MatchClouds::Request& req, ethzasl_icp_mapper::MatchClouds::Response& res)
{
	// get and check reference
	const DP referenceCloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(req.reference));
	const unsigned referenceGoodCount(referenceCloud.features.cols());
	const unsigned referencePointCount(req.reference.width * req.reference.height);
	const double referenceGoodRatio(double(referenceGoodCount) / double(referencePointCount));
	
	if (referenceGoodCount == 0)
	{
		ROS_ERROR("I found no good points in the reference cloud");
		return false;
	}
	if (referenceGoodRatio < 0.5)
	{
		ROS_WARN_STREAM("Partial reference cloud! Missing " << 100 - referenceGoodRatio*100.0 << "% of the cloud (received " << referenceGoodCount << ")");
	}
	
	// get and check reading
	const DP readingCloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(req.readings));
	const unsigned readingGoodCount(referenceCloud.features.cols());
	const unsigned readingPointCount(req.readings.width * req.readings.height);
	const double readingGoodRatio(double(readingGoodCount) / double(readingPointCount));
	
	if (readingGoodCount == 0)
	{
		ROS_ERROR("I found no good points in the reading cloud");
		return false;
	}
	if (readingGoodRatio < 0.5)
	{
		ROS_WARN_STREAM("Partial reference cloud! Missing " << 100 - readingGoodRatio*100.0 << "% of the cloud (received " << readingGoodCount << ")");
	}
	
	// check dimensions
	if (referenceCloud.features.rows() != readingCloud.features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: reference cloud is " << referenceCloud.features.rows()-1 << " while reading cloud is " << readingCloud.features.rows()-1);
		return false;
	}
	
	// call ICP
	try 
	{
		const PM::TransformationParameters transform(icp(readingCloud, referenceCloud));
		tf::transformTFToMsg(PointMatcher_ros::eigenMatrixToTransform<float>(transform), res.transform);
		ROS_INFO_STREAM("match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl);
	}
	catch (PM::ConvergenceError error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return false;
	}
	
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_matcher_service");
	ros::NodeHandle n;
	
	CloudMatcher matcher(n);
	
	ros::spin();
	
	return 0;
}

