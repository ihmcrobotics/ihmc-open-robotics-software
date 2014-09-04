#include <fstream>

#include <boost/version.hpp>

#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

using namespace std;
using namespace PointMatcherSupport;
using namespace visualization_msgs;
using namespace interactive_markers;

class MapMaintener
{
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PM::TransformationParameters TP;
	
	ros::NodeHandle& n;
	ros::NodeHandle& pn;

	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync;

	ros::Subscriber scanSub;
	ros::Subscriber cloudSub;
	ros::Publisher mapPub;
	
	// FIXME which service do we need?
	//ros::ServiceServer getPointMapSrv;
	//ros::ServiceServer saveMapSrv;
	//ros::ServiceServer resetSrv;

	PM::ICP icp;

	PM::DataPointsFilters inputFilters;
	PM::DataPointsFilters mapPreFilters;
	PM::DataPointsFilters mapPostFilters;
	unique_ptr<PM::Transformation> transformation;
	
	PM::DataPoints mapPointCloud;
	
	// Parameters
	string vtkFinalMapName; //!< name of the final vtk map
	int inputQueueSize; 
	const float size_x;
	const float size_y;
	const float size_z;

	// Interative markers
	boost::shared_ptr<InteractiveMarkerServer> server;
	MenuHandler menu_handler;
	bool mappingActive;
	bool singleScan;
	MenuHandler::EntryHandle h_single;
	MenuHandler::EntryHandle h_start; 
	MenuHandler::EntryHandle h_pause;
	MenuHandler::EntryHandle h_stop;
	MenuHandler::EntryHandle h_save;

	// tf
	tf::TransformListener tfListener;

public:
	boost::mutex publishLock;
	PM::TransformationParameters TObjectToMap;
	// Parameters
	string objectFrame;
	string mapFrame;
	
public:
	MapMaintener(ros::NodeHandle& n, ros::NodeHandle& pn);
	~MapMaintener();
	
protected:
	void gotCloud(const sensor_msgs::PointCloud2ConstPtr& cloudMsgIn, const nav_msgs::OdometryConstPtr& odom);
	void processCloud(DP cloud, const TP TScannerToMap);
	
	void makeMenuMarker( std::string name );
	void addRotAndTransCtrl(InteractiveMarker &int_marker, const double w, const double x, const double y, const double z, const std::string name);
	void update_tf(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );


	void singleScanCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void startMapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void pauseMapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void stopMapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void saveMapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

	//bool getPointMap(map_msgs::GetPointMap::Request &req, map_msgs::GetPointMap::Response &res);
	//bool saveMap(map_msgs::SaveMap::Request &req, map_msgs::SaveMap::Response &res);
	//bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

MapMaintener::MapMaintener(ros::NodeHandle& n, ros::NodeHandle& pn):
	n(n),
	pn(pn),
	cloud_sub(n, "cloud_in", 1),
	odom_sub(n, "icp_odom", 1),
	sync(MySyncPolicy(1), cloud_sub, odom_sub),
	transformation(PM::get().REG(Transformation).create("RigidTransformation")),
	vtkFinalMapName(getParam<string>("vtkFinalMapName", "finalMap_maintained.vtk")),
	size_x(getParam<double>("size_x", 1.0)),
	size_y(getParam<double>("size_y", 1.0)),
	size_z(getParam<double>("size_z", 1.0)),
	mappingActive(false),
	singleScan(false),
  tfListener(ros::Duration(30)),
	TObjectToMap(PM::TransformationParameters::Identity(4,4)),
	objectFrame(getParam<string>("object_frame", "map")),
	mapFrame(getParam<string>("map_frame", "map"))
{
	
	// load configs
	string configFileName;
	if (ros::param::get("~icpConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			icp.loadFromYaml(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load ICP config from YAML file " << configFileName);
			icp.setDefault();
		}
	}
	else
	{
		ROS_INFO_STREAM("No ICP config file given, using default");
		icp.setDefault();
	}

	if (ros::param::get("~inputFiltersConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			inputFilters = PM::DataPointsFilters(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load input filters config from YAML file " << configFileName);
		}
	}
	else
	{
		ROS_INFO_STREAM("No input filters config file given, not using these filters");
	}

	if (ros::param::get("~mapPreFiltersConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			mapPreFilters = PM::DataPointsFilters(ifs);

			// Add the bounding box
			PM::Parameters params;
			params["removeInside"] = toParam(0);
			params["xMin"] = toParam(-size_x/2);
			params["xMax"] = toParam(size_x/2);
			params["yMin"] = toParam(-size_y/2);
			params["yMax"] = toParam(size_y/2);
			params["zMin"] = toParam(-size_z/2);
			params["zMax"] = toParam(size_z/2);

			PM::DataPointsFilter* box = 
				PM::get().DataPointsFilterRegistrar.create("BoundingBoxDataPointsFilter", params);
			mapPreFilters.push_back(box);	

		}
		else
		{
			ROS_ERROR_STREAM("Cannot load map pre-filters config from YAML file " << configFileName);
		}
	}
	else
	{
		ROS_INFO_STREAM("No map pre-filters config file given, not using these filters");
	}

	if (ros::param::get("~mapPostFiltersConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			mapPostFilters = PM::DataPointsFilters(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load map post-filters config from YAML file " << configFileName);
		}
	}
	else
	{
		ROS_INFO_STREAM("No map post-filters config file given, not using these filters");
	}
	
	// topics and services initialization
	sync.registerCallback(boost::bind(&MapMaintener::gotCloud, this, _1, _2));

	//cloudSub = n.subscribe("cloud_in", inputQueueSize, &MapMaintener::gotCloud, this);
	mapPub = n.advertise<sensor_msgs::PointCloud2>("object_map", 2, true);
	
	//getPointMapSrv = n.advertiseService("dynamic_point_map", &MapMaintener::getPointMap, this);
	//saveMapSrv = pn.advertiseService("save_map", &MapMaintener::saveMap, this);
	//resetSrv = pn.advertiseService("reset", &MapMaintener::reset, this);

	// Setup interactive maker
  server.reset( new InteractiveMarkerServer("MapCenter","", false) );

	h_single = menu_handler.insert( "Record single scan", boost::bind(&MapMaintener::singleScanCallback, this, _1));
	h_start = menu_handler.insert( "Start object mapping", boost::bind(&MapMaintener::startMapCallback, this, _1));
	h_pause = menu_handler.insert( "Pause object mapping", boost::bind(&MapMaintener::pauseMapCallback, this, _1));
  h_stop = menu_handler.insert( "Stop/reset object mapping", boost::bind(&MapMaintener::stopMapCallback, this, _1));
	menu_handler.setVisible(h_stop, false);
	h_save = menu_handler.insert( "Save object to VTK", boost::bind(&MapMaintener::saveMapCallback, this, _1));

  makeMenuMarker( "object_menu" );
  menu_handler.apply( *server, "object_menu" );
	menu_handler.reApply( *server);
  server->applyChanges();
	
}

MapMaintener::~MapMaintener()
{
	server.reset();

	// save point cloud
	if (mapPointCloud.features.cols())
	{
		mapPointCloud.save(vtkFinalMapName);
	}
}


// Callback
void MapMaintener::gotCloud(const sensor_msgs::PointCloud2ConstPtr& cloudMsgIn, const nav_msgs::OdometryConstPtr& odom)
{
	// IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)

	if(mappingActive || singleScan)
	{
		// Convert pointcloud2 to DataPoint
		DP cloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*cloudMsgIn));
		// Convert odometry msg to TransformationParameters
		TP TScannerToMap = 	PointMatcher_ros::odomMsgToEigenMatrix<float>(*odom);
		processCloud(cloud, TScannerToMap);
	}

	singleScan = false;
}

// Point cloud processing
void MapMaintener::processCloud(DP newPointCloud, const TP TScannerToMap)
{
	string reason;
	timer t;
	
	// Convert point cloud
	const size_t goodCount(newPointCloud.features.cols());
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	ROS_INFO("Processing new point cloud");
	{
		timer t; // Print how long take the algo
		
		// Apply filters to incoming cloud, in scanner coordinates
		inputFilters.apply(newPointCloud);
		
		ROS_INFO_STREAM("Input filters took " << t.elapsed() << " [s]");
	}
	
	
	// Correct new points using ICP result and move them in their own frame
	//cout << "TObjectToMap: " << endl << TObjectToMap << endl;
	//cout << "TScannerToMap: " << endl << TScannerToMap << endl;
	//cout << "concat: " << endl << TObjectToMap.inverse() * TScannerToMap << endl;
	newPointCloud = transformation->compute(newPointCloud, transformation->correctParameters(TObjectToMap.inverse() * TScannerToMap)); 
	
	// Preparation of cloud for inclusion in map
	mapPreFilters.apply(newPointCloud);
	
	// FIXME put that as parameter
	if(newPointCloud.features.cols() < 400)
		return;

	// Generate first map
	if(!mapPointCloud.features.rows())	
	{
		mapPointCloud = newPointCloud;
		return;
	}
	

	// Check dimension
	if (newPointCloud.features.rows() != mapPointCloud.features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << newPointCloud.features.rows()-1 << " while map is " << mapPointCloud.features.rows()-1);
		return;
	}


	PM::TransformationParameters Tcorr;
	try
	{
		Tcorr = icp(newPointCloud, mapPointCloud);
		TObjectToMap = TObjectToMap * Tcorr.inverse();
	}
	catch (PM::ConvergenceError error)
	{
		ROS_WARN_STREAM("ICP failed to converge: " << error.what());
		return;
	}


	const double estimatedOverlap = icp.errorMinimizer->getOverlap();
	if(estimatedOverlap < 0.40)
	{
		ROS_WARN_STREAM("Estimated overlap too small: " << estimatedOverlap);
		return;
	}

	ROS_DEBUG_STREAM("Tcorr:\n" << Tcorr);

	cout << "Tcorr: " << endl << Tcorr << endl;
	newPointCloud = transformation->compute(newPointCloud, Tcorr); 
	// Merge point clouds to map
	mapPointCloud.concatenate(newPointCloud);

	// Map maintenance
	mapPostFilters.apply(mapPointCloud);
	
	ROS_INFO_STREAM("New map available (" << mapPointCloud.features.cols() << " pts), update took " << t.elapsed() << " [s]");

	// Publish map point cloud
	// FIXME this crash when used without descriptor
	if (mapPub.getNumSubscribers())
		mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud, objectFrame, ros::Time::now()));

	ROS_INFO_STREAM("Total map merging: " << t.elapsed() << " [s]");

	//ros::Rate r(2);
	//r.sleep();
}



// SERVICES definitions
//bool MapMaintener::getPointMap(map_msgs::GetPointMap::Request &req, map_msgs::GetPointMap::Response &res)
//{
//	if (!mapPointCloud)
//		return false;
//	
//	res.map = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud, mapFrame, ros::Time::now());
//	return true;
//}
//
//bool MapMaintener::saveMap(map_msgs::SaveMap::Request &req, map_msgs::SaveMap::Response &res)
//{
//	if (!mapPointCloud)
//		return false;
//	
//	try
//	{
//		mapPointCloud->save(req.filename.data);
//	}
//	catch (const std::runtime_error& e)
//	{
//		return false;
//	}
//	
//	return true;
//}
//
//bool MapMaintener::reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
//{
//	// note: no need for locking as we do ros::spin(), to update if we go for multi-threading
//	TOdomToMap = PM::TransformationParameters::Identity(4,4);
//	icp.clearMap();
//	return true;
//}

//================================================
// Markers
void MapMaintener::makeMenuMarker( std::string name )
{
	InteractiveMarker int_marker;

	// Header
	// TODO: adapt that for real system
	int_marker.header.frame_id = mapFrame;
	int_marker.pose.position.x = 0.0;
	int_marker.pose.position.y = 0.0;
	int_marker.pose.position.z = 0.0;

	int_marker.scale = 1;

 	// Information
	int_marker.name = name;
	//int_marker.description = "Move to the zone of interest";
	
	// Create 6 DoF control axis
	addRotAndTransCtrl(int_marker, 1, 1, 0, 0, "x");
	addRotAndTransCtrl(int_marker, 1, 0, 1, 0, "z");
	addRotAndTransCtrl(int_marker, 1, 0, 0, 1, "y");

	// Create a gray box to support a menu
	Marker grayBox;
  grayBox.type = Marker::CUBE;
  grayBox.scale.x = size_x;
  grayBox.scale.y = size_y;
  grayBox.scale.z = size_z;
  grayBox.color.r = 0.5;
  grayBox.color.g = 0.5;
  grayBox.color.b = 0.5;
  grayBox.color.a = 0.5;

	InteractiveMarkerControl control;
	control = InteractiveMarkerControl();
  control.markers.push_back(grayBox);
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
	int_marker.controls.push_back(control);
	int_marker.scale = std::max(std::max(size_x, size_y), size_z);

  server->insert( int_marker );
	server->setCallback(int_marker.name, boost::bind(&MapMaintener::update_tf, this, _1), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
}

void MapMaintener::addRotAndTransCtrl(InteractiveMarker &int_marker, const double w, const double x, const double y, const double z, const std::string name)
{
	InteractiveMarkerControl control;
	control.orientation.w = w;
	control.orientation.x = x;
	control.orientation.y = y;
	control.orientation.z = z;
	//control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
	control.name = "rotate_" + name;

  int_marker.controls.push_back( control );

}

// Marker callback
void MapMaintener::update_tf(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	//InteractiveMarker int_marker;
	//server->get("object_menu", int_marker);

	if(mapPointCloud.features.cols() == 0)
	{	
		nav_msgs::Odometry odom;
		odom.pose.pose = feedback->pose;
		publishLock.lock();
		TObjectToMap = 	PointMatcher_ros::odomMsgToEigenMatrix<float>(odom);
		publishLock.unlock();
		
		//int_marker.description = "Move to the zone of interest";
		menu_handler.reApply( *server);
		server->applyChanges();
	}
	else
	{
		//int_marker.description = "Reset the map to apply change";
		menu_handler.reApply( *server);
		server->applyChanges();
	}
}

void MapMaintener::singleScanCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
		singleScan = true;
}

void MapMaintener::startMapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	menu_handler.setVisible(h_start, false);
	menu_handler.setVisible(h_pause, true);
	menu_handler.setVisible(h_stop, true);
	menu_handler.reApply( *server);
  server->applyChanges();
	mappingActive = true;
}

void MapMaintener::pauseMapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	menu_handler.setVisible(h_start, true);
	menu_handler.setVisible(h_pause, false);
	menu_handler.setVisible(h_stop, true);
	menu_handler.reApply( *server);
  server->applyChanges();
	mappingActive = false;
}

void MapMaintener::stopMapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	menu_handler.setVisible(h_start, true);
	menu_handler.setVisible(h_pause, false);
	menu_handler.setVisible(h_stop, false);
	menu_handler.reApply( *server);
  server->applyChanges();
	mappingActive = false;

	mapPointCloud = PM::DataPoints();
	update_tf(feedback);
}

void MapMaintener::saveMapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	// save point cloud
	if (mapPointCloud.features.cols())
	{
		mapPointCloud.save(vtkFinalMapName);
	}
}

// Main function supporting the MapMaintener class
int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_maintainer");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	MapMaintener map(n, pn);

	tf::TransformBroadcaster tfBroadcaster;

	ros::Rate r(50);
	while(ros::ok())
	{
		map.publishLock.lock();
		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(map.TObjectToMap,  map.mapFrame, map.objectFrame, ros::Time::now()));
		map.publishLock.unlock();
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
