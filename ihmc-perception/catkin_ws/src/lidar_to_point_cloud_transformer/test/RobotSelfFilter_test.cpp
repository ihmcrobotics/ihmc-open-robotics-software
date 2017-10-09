/*
 * RobotSelfFilters_test.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: unknownid
 */

#include <lidar_to_point_cloud_transformer/RobotSelfFilter.hpp>
#include <string>
#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <exception>

#include <ros/ros.h>

void publishToROS(pcl::PointCloud<pcl::PointXYZI>& cloud, pcl::PointCloud<pcl::PointXYZI>& filteredCloud)
{

	ros::NodeHandle nh;
	ros::Publisher pubCloud = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("cloud",1,true);
	ros::Publisher pubFiltered= nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("filtered",1,true);

	ros::Rate r(10);
	while(ros::ok())
	{
		pubCloud.publish(cloud);
		pubFiltered.publish(filteredCloud);
		ros::spinOnce();
		r.sleep();
	}
}

void generateRandomCloud(pcl::PointCloud<pcl::PointXYZI>& cloud_out, int n,  double min, double max, std::string frame)
{
	cloud_out.height=1;
	cloud_out.width=n;
	cloud_out.resize(cloud_out.width*cloud_out.height);
	cloud_out.header.frame_id=frame;

	//A cube [0-1]^3
	srand(1000);
	for(int i=0;i<cloud_out.points.size();i++)
	{
	    cloud_out.points[i].x =  (max-min) * rand () / (RAND_MAX + 1.0f)+ min;
	    cloud_out.points[i].y =  (max-min) * rand () / (RAND_MAX + 1.0f)+ min;
	    cloud_out.points[i].z =  (max-min) * rand () / (RAND_MAX + 1.0f)+ min;
	}
}

TEST(TestSuite, frameLessCloud)
{
	pcl::PointCloud<pcl::PointXYZI> cloud;
    robot_model_loader::RobotModelLoader  model("robot_description");
    EXPECT_TRUE(model.getModel()!=NULL);

	RobotSelfFilter filter(model, 0.0);
    pcl::PointCloud<pcl::PointXYZI> filteredCloud;

    EXPECT_THROW( filter.filterPointClould(cloud, filteredCloud), std::runtime_error);
}

TEST(TestSuite, simpleTest)
{
	//generate fake point clouds
	pcl::PointCloud<pcl::PointXYZI> cloud;
	generateRandomCloud(cloud, 1000, 0.0, 2.0, "link"); //same as link frame, so we don't need to publish another tf.

    robot_model_loader::RobotModelLoader  model("robot_description");
    EXPECT_TRUE(model.getModel()!=NULL);

	RobotSelfFilter filter(model, 0.0);
    pcl::PointCloud<pcl::PointXYZI> filteredCloud;
	filter.filterPointClould(cloud, filteredCloud);

	pcl::PointXYZI origin;
	origin.x=origin.y=origin.z=0.5;
	for(int i=0;i<filteredCloud.points.size();i++)
	{
		EXPECT_GT(pcl::euclideanDistance(origin, filteredCloud.points[i]), 0.5);

	}
	std::cerr << "keeping " << filteredCloud.size()<< "/" << cloud.size() << "point" << std::endl;

	EXPECT_TRUE(filteredCloud.size()==932);


	//to visualize, uncomment following line and then   roslaunch lidar_to_point_cloud_transformer sphere.launch
	publishToROS(cloud, filteredCloud);
}


int main(int argc, char**argv)
{
	ros::init(argc, argv, "testnode");

	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


