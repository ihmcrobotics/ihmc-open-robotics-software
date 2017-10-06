#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include "ros/ros.h"
#include "ros/console.h"

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"

// Services
#include "map_msgs/SaveMap.h"
#include "ethzasl_icp_mapper/LoadMap.h"
#include "ethzasl_icp_mapper/CorrectPose.h"
#include "ethzasl_icp_mapper/SetMode.h"
#include "ethzasl_icp_mapper/GetMode.h"
#include "ethzasl_icp_mapper/GetBoundedMap.h"

#include "boost/algorithm/string.hpp"
#include "boost/filesystem.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/lexical_cast.hpp"

// ugly test depending on roscpp because tf_conversions is not properly versionized
#if !ROS_VERSION_MINIMUM(1, 9, 30)
	#define transformTFToEigen TransformTFToEigen
	#define transformEigenToTF TransformEigenToTF
#endif // !ROS_VERSION_MINIMUM(1, 9, 30)



using namespace std;
using namespace PointMatcherSupport;
using namespace visualization_msgs;
using namespace interactive_markers;
namespace fs = ::boost::filesystem;

class InteractMapper
{
public:
	typedef PointMatcher<float> PM;

	ros::NodeHandle& n;
	ros::NodeHandle& pn;

	int states_updatePose;

	// Parameters
	const string mapPath;
	const string baseFrame;
	const string odomFrame;
	const string mapFrame;
	
	// Publisher
	ros::Publisher test_mapPub;

	// Services
	ros::ServiceClient saveMapClient;
	ros::ServiceClient loadMapClient;
	ros::ServiceClient correctMapClient;
	ros::ServiceClient setModeClient;
	ros::ServiceClient getModeClient;
	// test
	ros::ServiceClient getBoundedMapClient;

	// Interative markers
	boost::shared_ptr<InteractiveMarkerServer> server;
	MenuHandler menu_handler;
	MenuHandler::EntryHandle h_localize;
	MenuHandler::EntryHandle h_map;
	MenuHandler::EntryHandle h_load;
	MenuHandler::EntryHandle h_save;
	MenuHandler::EntryHandle h_adjustPose;
	MenuHandler::EntryHandle h_getboundedMap;

	tf::TransformListener tfListener;
	geometry_msgs::Pose markerPose;

	InteractMapper(ros::NodeHandle& n, ros::NodeHandle& pn);
	~InteractMapper();

protected:
	void localizeCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void mapCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void saveMapCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void loadMapCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void adjustPoseCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	void getBoundedMapCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
	
	void update_tf(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

private:
	void makeMenuMarker(string name);
	void addRotAndTransCtrl(InteractiveMarker &int_marker, const double w, const double x, const double y, const double z, const std::string name);
};


InteractMapper::InteractMapper(ros::NodeHandle& n, ros::NodeHandle& pn):
	n(n),
	pn(pn),
	mapPath(getParam<string>("map_path", "/home/frank/.ros/3D_maps")),
	baseFrame(getParam<string>("base_frame", "base_link")),
	odomFrame(getParam<string>("odom_frame", "odom")),
	mapFrame(getParam<string>("map_frame", "map")),
  tfListener(ros::Duration(30))
{
	ROS_INFO_STREAM("Waiting for a mapper node...");
	ros::service::waitForService("mapper/set_mode");
	ROS_INFO_STREAM("Mapper node found");
	
	states_updatePose = 0;

	// Publisher
	test_mapPub = n.advertise<sensor_msgs::PointCloud2>("debug_bounded_map", 2, true);

	// Service setups
	// TODO: remove private tags
	saveMapClient = n.serviceClient<map_msgs::SaveMap>("mapper/save_map");
	loadMapClient = n.serviceClient<ethzasl_icp_mapper::LoadMap>("mapper/load_map");
	correctMapClient = n.serviceClient<ethzasl_icp_mapper::CorrectPose>("mapper/correct_pose");
	setModeClient = n.serviceClient<ethzasl_icp_mapper::SetMode>("mapper/set_mode");
	getModeClient = n.serviceClient<ethzasl_icp_mapper::GetMode>("mapper/get_mode");
	getBoundedMapClient = n.serviceClient<ethzasl_icp_mapper::GetBoundedMap>("mapper/get_bounded_map");

	// Setup interactive maker
	server.reset( new InteractiveMarkerServer("MapControl","", false) );
	h_localize = menu_handler.insert( "Localize", boost::bind(&InteractMapper::localizeCallback, this, _1));
	h_map = menu_handler.insert( "Map", boost::bind(&InteractMapper::mapCallback, this, _1));
	h_save = menu_handler.insert( "Save map to VTK", boost::bind(&InteractMapper::saveMapCallback, this, _1));
	h_load = menu_handler.insert( "Load map from VTK");

	h_adjustPose = menu_handler.insert( "Correct map pose", boost::bind(&InteractMapper::adjustPoseCallback, this, _1));
	
	h_getboundedMap = menu_handler.insert( "Publish 3x3 meter map", boost::bind(&InteractMapper::getBoundedMapCallback, this, _1));
	
	// Fetch states of the mapper node
	ethzasl_icp_mapper::GetMode srv;
	getModeClient.call(srv);
	
	if(srv.response.localize)
		menu_handler.setCheckState(h_localize, MenuHandler::CHECKED);
	else
		menu_handler.setCheckState(h_localize, MenuHandler::UNCHECKED);
	if(srv.response.map)
		menu_handler.setCheckState(h_map, MenuHandler::CHECKED);
	else
		menu_handler.setCheckState(h_map, MenuHandler::UNCHECKED);
	
	menu_handler.setCheckState(h_adjustPose, MenuHandler::UNCHECKED);

	makeMenuMarker( "map_menu" );
  
	// Generate load menu based on map_path
	fs::path path(mapPath);
	if(!(fs::exists(path) && fs::is_directory(path)))
	{
		ROS_ERROR_STREAM("Path " << mapPath << " does not exist");
	}
	else
	{
		fs::directory_iterator end_iter;
		for ( fs::directory_iterator dir_itr( path );
					dir_itr != end_iter;
					++dir_itr )
		{
			try
			{
				if ( fs::is_regular_file( dir_itr->status()) )
				{
					if(dir_itr->path().extension() == ".vtk")
					{
						#if BOOST_FILESYSTEM_VERSION >= 3
						menu_handler.insert(h_load, dir_itr->path().filename().string(), boost::bind(&InteractMapper::loadMapCallback, this, _1) );
						#else
						menu_handler.insert(h_load, dir_itr->path().filename(), boost::bind(&InteractMapper::loadMapCallback, this, _1) );
						#endif
					}
				}
			}
			catch ( const std::exception & ex )
			{
				std::cout << dir_itr->path().filename() << " " << ex.what() << std::endl;
			}
		}
	}

	menu_handler.apply( *server, "map_menu" );
	menu_handler.reApply( *server);
  server->applyChanges();

}

InteractMapper::~InteractMapper()
{
	
}


//================================================
// Callbacks
void InteractMapper::localizeCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ethzasl_icp_mapper::SetMode srv;

	MenuHandler::CheckState state_loc;
	MenuHandler::CheckState state_map;
	menu_handler.getCheckState(h_localize, state_loc);
	menu_handler.getCheckState(h_map, state_map);

	if(state_loc == MenuHandler::CHECKED)
	{
		menu_handler.setCheckState(h_localize, MenuHandler::UNCHECKED);
		menu_handler.setCheckState(h_map, MenuHandler::UNCHECKED);
		srv.request.localize = false;
		srv.request.map = false;
	}
	else
	{
		menu_handler.setCheckState(h_localize, MenuHandler::CHECKED);
		menu_handler.setCheckState(h_map, MenuHandler::UNCHECKED);
		srv.request.localize = true;
		srv.request.map = false;
	}
	
	menu_handler.reApply( *server );
	server->applyChanges();
	
	srv.request.applyChange = true;
	setModeClient.call(srv);
}

void InteractMapper::mapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ethzasl_icp_mapper::SetMode srv;

	MenuHandler::CheckState state_loc;
	MenuHandler::CheckState state_map;
	menu_handler.getCheckState(h_localize, state_loc);
	menu_handler.getCheckState(h_map, state_map);

	if(state_map == MenuHandler::CHECKED)
	{
		menu_handler.setCheckState(h_map, MenuHandler::UNCHECKED);
		srv.request.map = false;
		if(state_loc == MenuHandler::CHECKED)
			srv.request.localize = true;
		else
			srv.request.localize = false;
	}
	else
	{
		menu_handler.setCheckState(h_localize, MenuHandler::CHECKED);
		menu_handler.setCheckState(h_map, MenuHandler::CHECKED);
		srv.request.localize = true;
		srv.request.map = true;
	}
	
	menu_handler.reApply( *server );
	server->applyChanges();
	
	srv.request.applyChange = true;
	setModeClient.call(srv);
}

void InteractMapper::saveMapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	map_msgs::SaveMap srv;
	srv.request.filename.data = mapPath + "/default_map.vtk";
	saveMapClient.call(srv);
}

void InteractMapper::loadMapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ethzasl_icp_mapper::SetMode srv_states;
	menu_handler.setCheckState(h_localize, MenuHandler::UNCHECKED);
	menu_handler.setCheckState(h_map, MenuHandler::UNCHECKED);
	srv_states.request.localize = false;
	srv_states.request.map = false;
	setModeClient.call(srv_states);

	
	MenuHandler::EntryHandle handle = feedback->menu_entry_id;
	string filename;
	menu_handler.getTitle(handle, filename);

	ROS_INFO_STREAM("Loading " << filename);
	
	ethzasl_icp_mapper::LoadMap srv;
	srv.request.filename.data = mapPath + "/" + filename;
	loadMapClient.call(srv);

	menu_handler.reApply( *server );
  server->applyChanges();
}

void InteractMapper::adjustPoseCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	MenuHandler::CheckState state;
	menu_handler.getCheckState(h_adjustPose, state);
	
	if(state == MenuHandler::UNCHECKED)
	{
		cerr << "checked " << states_updatePose << endl;

		if(states_updatePose == 0)
			states_updatePose = 1;
		else
			states_updatePose = 0;

	}

	if(state == MenuHandler::CHECKED)
	{
		cerr << "unchecked " << states_updatePose << endl;

		if(states_updatePose == 1)
			states_updatePose = 2;
		else
			states_updatePose = 0;

	}
	
	if(states_updatePose == 2)
	{
		states_updatePose = 0;

		ROS_INFO_STREAM("Updating giving pose");

		nav_msgs::Odometry odom;
		odom.pose.pose = feedback->pose;
		markerPose = feedback->pose;

		const PM::TransformationParameters TMarkerToMap =
			PointMatcher_ros::odomMsgToEigenMatrix<float>(odom);

		const ros::Time stamp = ros::Time::now();

		tfListener.waitForTransform(mapFrame, baseFrame, stamp, ros::Duration(0.5));
		const PM::TransformationParameters TBaseToMap = 
			PointMatcher_ros::transformListenerToEigenMatrix<float>(
				tfListener,
				mapFrame,
				baseFrame,
				stamp
			);
			
		tfListener.waitForTransform(odomFrame, baseFrame, stamp, ros::Duration(0.5));
		const PM::TransformationParameters TBaseToOdom = 
			PointMatcher_ros::transformListenerToEigenMatrix<float>(
				tfListener,
				odomFrame,
				baseFrame,
				stamp
			);
		
	//	const PM::TransformationParameters TOdomToMap = 
	//		PointMatcher_ros::transformListenerToEigenMatrix<float>(
	//			tfListener,
	//			mapFrame,
	//			odomFrame,
	//			ros::Time::now()
	//		);
	//	
		const PM::TransformationParameters TCorr =
			 TMarkerToMap * TBaseToOdom.inverse();

		//cout << "TMarkerToMap:\n" << TMarkerToMap << endl;
		//cout << "TOdomToMap:\n" << TOdomToMap << endl;
		//cout << "TCorr:\n" << TCorr << endl;

		ethzasl_icp_mapper::CorrectPose srv;

		srv.request.odom = PointMatcher_ros::eigenMatrixToOdomMsg<float>(TCorr, mapFrame, ros::Time::now());

		correctMapClient.call(srv);

		//InteractiveMarker int_marker;
		//server->get("map_menu", int_marker);
		//makeMenuMarker("map_menu");
		
		
		menu_handler.reApply( *server );
		server->applyChanges();
		//ros::Duration(0.1).sleep();
	}

	if(state == MenuHandler::CHECKED)
		menu_handler.setCheckState(h_adjustPose, MenuHandler::UNCHECKED);
	else
	{
		menu_handler.setCheckState(h_adjustPose, MenuHandler::CHECKED);
		
		ethzasl_icp_mapper::SetMode srv_states;
		menu_handler.setCheckState(h_localize, MenuHandler::UNCHECKED);
		menu_handler.setCheckState(h_map, MenuHandler::UNCHECKED);
		srv_states.request.localize = false;
		srv_states.request.map = false;
		setModeClient.call(srv_states);
	}
	
	menu_handler.reApply( *server );
  server->applyChanges();
}

void InteractMapper::update_tf(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{	
	MenuHandler::CheckState state;
	menu_handler.getCheckState(h_adjustPose, state);
	

	
}

void InteractMapper::getBoundedMapCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ethzasl_icp_mapper::GetBoundedMap srv;
	srv.request.topRightCorner.x = 1.5;
	srv.request.topRightCorner.y = 1.5;
	srv.request.topRightCorner.z = 1.5;

	srv.request.bottomLeftCorner.x = -1.5;
	srv.request.bottomLeftCorner.y = -1.5;
	srv.request.bottomLeftCorner.z = -1.5;

	// FIXME: throw exeception: finish debug here...
	tf::StampedTransform stampedTr;
	ros::Time stamp = ros::Time::now();
	tfListener.waitForTransform(mapFrame, baseFrame, stamp, ros::Duration(2));
	tfListener.lookupTransform(mapFrame, baseFrame, stamp, stampedTr);
	
	Eigen::Affine3d eigenTr;
	tf::transformTFToEigen(stampedTr, eigenTr);

	srv.request.mapCenter.orientation.x = eigenTr.rotation().x();
	srv.request.mapCenter.orientation.z = eigenTr.rotation().y();
	srv.request.mapCenter.orientation.y = eigenTr.rotation().z();
	srv.request.mapCenter.orientation.w = eigenTr.rotation().w();
	srv.request.mapCenter.position.x = eigenTr.translation().x();
	srv.request.mapCenter.position.y = eigenTr.translation().y();
	srv.request.mapCenter.position.z = eigenTr.translation().z();
	getBoundedMapClient.call(srv);

	test_mapPub.publish(srv.response.boundedMap);

}


//================================================
// Markers
void InteractMapper::makeMenuMarker(string name)
{
	InteractiveMarker int_marker;

	// Header
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
  grayBox.type = Marker::SPHERE;
  grayBox.scale.x = 2.5;
  grayBox.scale.y = 2.5;
  grayBox.scale.z = 2.5;

  grayBox.color.r = 0.95;
  grayBox.color.g = 0.95;
  grayBox.color.b = 0.85;
  grayBox.color.a = 0.5;

	InteractiveMarkerControl control;
	control = InteractiveMarkerControl();
  control.markers.push_back(grayBox);
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
	int_marker.controls.push_back(control);

	int_marker.scale = 2.5;

  server->insert( int_marker );
	server->setCallback(int_marker.name, boost::bind(&InteractMapper::update_tf, this, _1), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
}

void InteractMapper::addRotAndTransCtrl(InteractiveMarker &int_marker, const double w, const double x, const double y, const double z, const std::string name)
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

// Main function supporting the InteractMapper class
int main(int argc, char **argv)
{
	ros::init(argc, argv, "interact_mapper");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	InteractMapper interact(n, pn);
	
	tf::StampedTransform stampedTr;

	ros::Rate r(10);
	while(ros::ok())
	{
		if(interact.states_updatePose != 1)
		{
			ros::Time stamp = ros::Time::now();
			if (interact.tfListener.waitForTransform(interact.mapFrame, interact.baseFrame, stamp, ros::Duration(1)))
			{
				interact.tfListener.lookupTransform(interact.mapFrame, interact.baseFrame, stamp, stampedTr);

				InteractiveMarker int_marker;
				interact.server->get("map_menu", int_marker);
				int_marker.pose.position.x = stampedTr.getOrigin().x();
				int_marker.pose.position.y = stampedTr.getOrigin().y();
				int_marker.pose.position.z = stampedTr.getOrigin().z();
				int_marker.pose.orientation.x = stampedTr.getRotation().x();
				int_marker.pose.orientation.y = stampedTr.getRotation().y();
				int_marker.pose.orientation.z = stampedTr.getRotation().z();
				int_marker.pose.orientation.w = stampedTr.getRotation().w();

				interact.server->insert(int_marker);
				interact.server->applyChanges();
			}
			else
			{
				ROS_WARN_STREAM("Cannot find transformation from " << interact.mapFrame << " to " << interact.baseFrame);
			}
		}
		

		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
