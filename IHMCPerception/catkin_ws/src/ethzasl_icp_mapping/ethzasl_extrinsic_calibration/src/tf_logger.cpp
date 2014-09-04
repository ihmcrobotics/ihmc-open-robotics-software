#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;

/*const string baseLinkFrame = "/base_link";
const string odomFrame = "/odom";
const string kinectFrame = "/openni_rgb_optical_frame";
const string worldFrame = "/world";
*/

// see http://www.ros.org/wiki/Clock for how to manage timing 
/*
	Execute the following to use this program:
		roscore
		rosparam set /use_sim_time true
		rosbag play ../data/2011-01-13-13-56-40.bag --clock
		bin/logger
*/

ostream& operator<< (ostream& os, const tf::Quaternion& quat)
{
	os << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w();
	return os;
}

ostream& operator<< (ostream& os, const tf::Vector3& trans)
{
	os << trans.x() << " " << trans.y() << " " << trans.z();
	return os;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_logger");

	ros::NodeHandle node("~");
	
	string baseLinkFrame;
	node.param<string>("baseLinkFrame", baseLinkFrame, "/base_link");
	string odomFrame;
	node.param<string>("odomFrame", odomFrame, "/odom");
	string kinectFrame;
	node.param<string>("kinectFrame", kinectFrame, "/openni_rgb_optical_frame");
	string worldFrame;
	node.param<string>("worldFrame", worldFrame, "/world");
	string outputFileName;
	node.param<string>("outputFileName", outputFileName, "output.txt");
	cout << baseLinkFrame << " " << odomFrame << " " << kinectFrame << " " <<
			worldFrame << " " << outputFileName << endl;

	tf::TransformListener t(ros::Duration(20));
	tf::StampedTransform tr_o, tr_i;
	
	ROS_INFO_STREAM("waiting for initial transforms");
	while (node.ok())
	{
		ros::Time now(ros::Time::now());
		//ROS_INFO_STREAM(now);
		if (t.waitForTransform(baseLinkFrame, now, baseLinkFrame, now, odomFrame, ros::Duration(0.1)))
			break;
		//ROS_INFO("wait");
		//ros::Duration(0.1).sleep();
	}
	ROS_INFO_STREAM("got first odom to baseLink");
	while (node.ok())
	{
		ros::Time now(ros::Time::now());
		//ROS_INFO_STREAM(now);
		if (t.waitForTransform(kinectFrame, now, kinectFrame, now, worldFrame, ros::Duration(0.1)))
			break;
		//ROS_INFO("wait");
		//ros::Duration(0.1).sleep();
	}
	ROS_INFO_STREAM("got first world to kinect");
	
	sleep(3);
	
	ros::Rate rate(0.5);
	ofstream ofs(outputFileName.c_str());
	while (node.ok())
	{
		// sleep
		ros::spinOnce();
		rate.sleep();
		
		// get parameters from transforms
		ros::Time curTime(ros::Time::now());
		ros::Time lastTime = curTime - ros::Duration(2);
		//ROS_INFO_STREAM("curTime: " << curTime << ", lastTime: " << lastTime);
		if (!t.waitForTransform(baseLinkFrame, curTime, baseLinkFrame, lastTime, odomFrame, ros::Duration(3)))
			break;
		if (!t.waitForTransform(kinectFrame, curTime, kinectFrame, lastTime, worldFrame, ros::Duration(3)))
			break;
		ofs << curTime << " ";
		t.lookupTransform(baseLinkFrame, curTime, baseLinkFrame, lastTime, odomFrame, tr_o);
		ofs << tr_o.getOrigin() << " " << tr_o.getRotation() << " ";
		t.lookupTransform(kinectFrame, curTime, kinectFrame, lastTime, worldFrame, tr_i);
		ofs << tr_i.getOrigin() << " " << tr_i.getRotation() << endl;
	}
	
	return 0;
}
