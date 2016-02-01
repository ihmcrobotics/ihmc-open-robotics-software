#include <fstream>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"


using namespace std;
using namespace PointMatcherSupport;

class PublishVTK
{
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PointMatcherIO<float> PMIO;

	ros::NodeHandle& n;

	// Parameters
	const string mapFrame;
	const string cloudTopic;
	const string csvListFiles;
	const string dataDirectory;
	const string singleFile;
	const double publishRate;
	const bool pauseEachMsg;

	ros::Publisher cloudPub;
	PMIO::FileInfoVector list;
	unsigned currentId;

public:
	PublishVTK(ros::NodeHandle& n);
	void publish();
	void run();
};

PublishVTK::PublishVTK(ros::NodeHandle& n):
	n(n),
	mapFrame(getParam<string>("mapFrameId", "/map")),
	cloudTopic(getParam<string>("cloudTopic", "/point_cloud")),
	csvListFiles(getParam<string>("csvListFiles", "")),
	dataDirectory(getParam<string>("dataDirectory", "")),
	singleFile(getParam<string>("singleFile", "")),
	publishRate(getParam<double>("publishRate", 1.0)),
	pauseEachMsg(getParam<bool>("pauseEachMsg", false))
{
	// ROS initialization
	cloudPub = n.advertise<sensor_msgs::PointCloud2>(cloudTopic, 1);

	if(csvListFiles != "")
	{
		list = PMIO::FileInfoVector(csvListFiles, dataDirectory);
	}
	currentId = 0;
}

void PublishVTK::publish()
{
	if (cloudPub.getNumSubscribers())
	{

		DP cloud;
		if(singleFile != "")
			cloud = DP::load(singleFile);
		else
		{
			if(csvListFiles != "")
			{
				if(pauseEachMsg)
				{
					cout << endl << "Press <ENTER> to continue or <CTRL-c>  to exit" << endl;
					cin.clear();
					cin.ignore(INT_MAX, '\n');
				}

				ROS_INFO_STREAM("Publishing file [" << currentId << "/" << list.size() << "]: " << list[currentId].readingFileName);
				cloud = DP::load(list[currentId].readingFileName);
				currentId++;
				if(currentId >= list.size())
				{
					cout << endl << "Press <ENTER> to restart or <CTRL-c>  to exit" << endl;
					cin.clear();
					cin.ignore(INT_MAX, '\n');
					currentId = 0;
				}
			}
		}

		if(singleFile != "" || csvListFiles != "")
			cloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(cloud, mapFrame, ros::Time::now()));
		else
		{
			ROS_ERROR_STREAM("No files found");
			abort();
		}
	}
}

void PublishVTK::run()
{
	ros::Rate r(publishRate);
	while (ros::ok())
	{
		publish();
		r.sleep();
	}
}

// Main function supporting the ExportVtk class
int main(int argc, char **argv)
{
	ros::init(argc, argv, "VtkToPointCloud_node");
	ros::NodeHandle n;
	PublishVTK pub(n);
	pub.run();
	
	return 0;
}
