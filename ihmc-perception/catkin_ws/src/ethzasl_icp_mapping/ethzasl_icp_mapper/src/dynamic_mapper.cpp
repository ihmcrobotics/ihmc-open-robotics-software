#include <fstream>

#include <boost/version.hpp>
#include <boost/thread.hpp>
#if BOOST_VERSION >= 104100
	#include <boost/thread/future.hpp>
#endif // BOOST_VERSION >=  104100

#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "nabo/nabo.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"

// Services
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "map_msgs/GetPointMap.h"
#include "map_msgs/SaveMap.h"
#include "ethzasl_icp_mapper/LoadMap.h"
#include "ethzasl_icp_mapper/CorrectPose.h"
#include "ethzasl_icp_mapper/SetMode.h"
#include "ethzasl_icp_mapper/GetMode.h"
#include "ethzasl_icp_mapper/GetBoundedMap.h" // FIXME: should that be moved to map_msgs?

using namespace std;
using namespace PointMatcherSupport;

class Mapper
{
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PM::Matches Matches;
	
	typedef typename Nabo::NearestNeighbourSearch<float> NNS;
	typedef typename NNS::SearchType NNSearchType;
		
	ros::NodeHandle& n;
	ros::NodeHandle& pn;
	
	// Subscribers
	ros::Subscriber scanSub;
	ros::Subscriber cloudSub;
	
	// Publishers
	ros::Publisher mapPub;
	ros::Publisher outlierPub;
	ros::Publisher odomPub;
	ros::Publisher odomErrorPub;
	
	// Services
	ros::ServiceServer getPointMapSrv;
	ros::ServiceServer saveMapSrv;
	ros::ServiceServer loadMapSrv;
	ros::ServiceServer resetSrv;
	ros::ServiceServer correctPoseSrv;
	ros::ServiceServer setModeSrv;
	ros::ServiceServer getModeSrv;
	ros::ServiceServer getBoundedMapSrv;

	// Time
	ros::Time mapCreationTime;
	ros::Time lastPoinCloudTime;

	// libpointmatcher
	PM::DataPointsFilters inputFilters;
	PM::DataPointsFilters mapPreFilters;
	PM::DataPointsFilters mapPostFilters;
	PM::DataPoints *mapPointCloud;
	PM::ICPSequence icp;
	unique_ptr<PM::Transformation> transformation;
	
	// multi-threading mapper
	#if BOOST_VERSION >= 104100
	typedef boost::packaged_task<PM::DataPoints*> MapBuildingTask;
	typedef boost::unique_future<PM::DataPoints*> MapBuildingFuture;
	boost::thread mapBuildingThread;
	MapBuildingTask mapBuildingTask;
	MapBuildingFuture mapBuildingFuture;
	bool mapBuildingInProgress;
	#endif // BOOST_VERSION >= 104100
	bool processingNewCloud; 

	// Parameters
	bool publishMapTf; 
	bool useConstMotionModel; 
	bool localizing;
	bool mapping;
	int minReadingPointCount;
	int minMapPointCount;
	int inputQueueSize; 
	double minOverlap;
	double maxOverlapToMerge;
	double tfRefreshPeriod;  //!< if set to zero, tf will be publish at the rate of the incoming point cloud messages 
	string odomFrame;
	string mapFrame;
	string vtkFinalMapName; //!< name of the final vtk map

	const double mapElevation; // initial correction on z-axis //FIXME: handle the full matrix
	
	// Parameters for dynamic filtering
	const float priorStatic; //!< ratio. Prior to be static when a new point is added
	const float priorDyn; //!< ratio. Prior to be dynamic when a new point is added
	const float maxAngle; //!< in rad. Openning angle of a laser beam
	const float eps_a; //!< ratio. Error proportional to the laser distance
	const float eps_d; //!< in meter. Fix error on the laser distance
	const float alpha; //!< ratio. Propability of staying static given that the point was dynamic
	const float beta; //!< ratio. Propability of staying dynamic given that the point was static
	const float maxDyn; //!< ratio. Threshold for which a point will stay dynamic
	const float maxDistNewPoint; //!< in meter. Distance at which a new point will be added in the global map.


	

	PM::TransformationParameters TOdomToMap;
	boost::thread publishThread;
	boost::mutex publishLock;
	ros::Time publishStamp;
	
	tf::TransformListener tfListener;
	tf::TransformBroadcaster tfBroadcaster;

	const float eps;
	
public:
	Mapper(ros::NodeHandle& n, ros::NodeHandle& pn);
	~Mapper();
	
protected:
	void gotScan(const sensor_msgs::LaserScan& scanMsgIn);
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);
	void processCloud(unique_ptr<DP> cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);
	void processNewMapIfAvailable();
	void setMap(DP* newPointCloud);
	DP* updateMap(DP* newPointCloud, const PM::TransformationParameters Ticp, bool updateExisting);
	void waitForMapBuildingCompleted();
	
	void publishLoop(double publishPeriod);
	void publishTransform();
	
	// Services
	bool getPointMap(map_msgs::GetPointMap::Request &req, map_msgs::GetPointMap::Response &res);
	bool saveMap(map_msgs::SaveMap::Request &req, map_msgs::SaveMap::Response &res);
	bool loadMap(ethzasl_icp_mapper::LoadMap::Request &req, ethzasl_icp_mapper::LoadMap::Response &res);
	bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool correctPose(ethzasl_icp_mapper::CorrectPose::Request &req, ethzasl_icp_mapper::CorrectPose::Response &res);
	bool setMode(ethzasl_icp_mapper::SetMode::Request &req, ethzasl_icp_mapper::SetMode::Response &res);
	bool getMode(ethzasl_icp_mapper::GetMode::Request &req, ethzasl_icp_mapper::GetMode::Response &res);
	bool getBoundedMap(ethzasl_icp_mapper::GetBoundedMap::Request &req, ethzasl_icp_mapper::GetBoundedMap::Response &res);
};

Mapper::Mapper(ros::NodeHandle& n, ros::NodeHandle& pn):
	n(n),
	pn(pn),
	mapPointCloud(0),
	transformation(PM::get().REG(Transformation).create("RigidTransformation")),
	#if BOOST_VERSION >= 104100
	mapBuildingInProgress(false),
	#endif // BOOST_VERSION >= 104100
	processingNewCloud(false),
	publishMapTf(getParam<bool>("publishMapTf", true)),
	useConstMotionModel(getParam<bool>("useConstMotionModel", false)),
	localizing(getParam<bool>("localizing", true)),
	mapping(getParam<bool>("mapping", true)),
	minReadingPointCount(getParam<int>("minReadingPointCount", 2000)),
	minMapPointCount(getParam<int>("minMapPointCount", 500)),
	inputQueueSize(getParam<int>("inputQueueSize", 10)),
	minOverlap(getParam<double>("minOverlap", 0.5)),
	maxOverlapToMerge(getParam<double>("maxOverlapToMerge", 0.9)),
	tfRefreshPeriod(getParam<double>("tfRefreshPeriod", 0.01)),
	odomFrame(getParam<string>("odom_frame", "odom")),
	mapFrame(getParam<string>("map_frame", "map")),
	vtkFinalMapName(getParam<string>("vtkFinalMapName", "finalMap.vtk")),
	mapElevation(getParam<double>("mapElevation", 0)),
	priorStatic(getParam<double>("priorStatic", 0.5)),
	priorDyn(getParam<double>("priorDyn", 0.5)),
	maxAngle(getParam<double>("maxAngle", 0.02)),
	eps_a(getParam<double>("eps_a", 0.05)),
	eps_d(getParam<double>("eps_d", 0.02)),
	alpha(getParam<double>("alpha", 0.99)),
	beta(getParam<double>("beta", 0.99)),
	maxDyn(getParam<double>("maxDyn", 0.95)),
	maxDistNewPoint(pow(getParam<double>("maxDistNewPoint", 0.1),2)),
	TOdomToMap(PM::TransformationParameters::Identity(4, 4)),
	publishStamp(ros::Time::now()),
  tfListener(ros::Duration(30)),
	eps(0.0001)
{

	// Ensure proper states
	if(localizing == false)
		mapping = false;
	if(mapping == true)
		localizing = true;

	// set logger
	if (getParam<bool>("useROSLogger", false))
		PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);

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
	if (getParam<bool>("useROSLogger", false))
		PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);
	
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
	if (getParam<bool>("subscribe_scan", true))
		scanSub = n.subscribe("scan", inputQueueSize, &Mapper::gotScan, this);
	if (getParam<bool>("subscribe_cloud", true))
		cloudSub = n.subscribe("cloud_in", inputQueueSize, &Mapper::gotCloud, this);
	mapPub = n.advertise<sensor_msgs::PointCloud2>("point_map", 2, true);
	outlierPub = n.advertise<sensor_msgs::PointCloud2>("outliers", 2, true);
	odomPub = n.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
	odomErrorPub = n.advertise<nav_msgs::Odometry>("icp_error_odom", 50, true);
	getPointMapSrv = n.advertiseService("dynamic_point_map", &Mapper::getPointMap, this);
	saveMapSrv = pn.advertiseService("save_map", &Mapper::saveMap, this);
	loadMapSrv = pn.advertiseService("load_map", &Mapper::loadMap, this);
	resetSrv = pn.advertiseService("reset", &Mapper::reset, this);
	correctPoseSrv = pn.advertiseService("correct_pose", &Mapper::correctPose, this);
	setModeSrv = pn.advertiseService("set_mode", &Mapper::setMode, this);
	getModeSrv = pn.advertiseService("get_mode", &Mapper::getMode, this);
	getBoundedMapSrv = pn.advertiseService("get_bounded_map", &Mapper::getBoundedMap, this);

	// refreshing tf transform thread
	publishThread = boost::thread(boost::bind(&Mapper::publishLoop, this, tfRefreshPeriod));

}

Mapper::~Mapper()
{
	#if BOOST_VERSION >= 104100
	// wait for map-building thread
	if (mapBuildingInProgress)
	{
		mapBuildingFuture.wait();
		if (mapBuildingFuture.has_value())
			delete mapBuildingFuture.get();
	}
	#endif // BOOST_VERSION >= 104100
	// wait for publish thread
	publishThread.join();
	// save point cloud
	if (mapPointCloud)
	{
		mapPointCloud->save(vtkFinalMapName);
		delete mapPointCloud;
	}
}

void Mapper::gotScan(const sensor_msgs::LaserScan& scanMsgIn)
{
	if(localizing)
	{
		const ros::Time endScanTime(scanMsgIn.header.stamp + ros::Duration(scanMsgIn.time_increment * (scanMsgIn.ranges.size() - 1)));
		unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(scanMsgIn, &tfListener, odomFrame)));
		processCloud(move(cloud), scanMsgIn.header.frame_id, endScanTime, scanMsgIn.header.seq);
	}
}

void Mapper::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	if(localizing)
	{
		unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
		processCloud(move(cloud), cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
	}
}

struct BoolSetter
{
public:
	bool toSetValue;
	BoolSetter(bool& target, bool toSetValue):
		toSetValue(toSetValue),
		target(target)
	{}
	~BoolSetter()
	{
		target = toSetValue;
	}
protected:
	bool& target;
};

void Mapper::processCloud(unique_ptr<DP> newPointCloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq)
{
	processingNewCloud = true;
	BoolSetter stopProcessingSetter(processingNewCloud, false);

	// if the future has completed, use the new map
	processNewMapIfAvailable();
	cerr << "received new map" << endl;
	
	// IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)
	timer t;


	
	
	// Convert point cloud
	const size_t goodCount(newPointCloud->features.cols());
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	// Dimension of the point cloud, important since we handle 2D and 3D
	const int dimp1(newPointCloud->features.rows());

	if(!(newPointCloud->descriptorExists("stamps_Msec") && newPointCloud->descriptorExists("stamps_sec") && newPointCloud->descriptorExists("stamps_nsec")))
	{
		const float Msec = round(stamp.sec/1e6);
		const float sec = round(stamp.sec - Msec*1e6);
		const float nsec = round(stamp.nsec);

		const PM::Matrix desc_Msec = PM::Matrix::Constant(1, goodCount, Msec);
		const PM::Matrix desc_sec = PM::Matrix::Constant(1, goodCount, sec);
		const PM::Matrix desc_nsec = PM::Matrix::Constant(1, goodCount, nsec);
		newPointCloud->addDescriptor("stamps_Msec", desc_Msec);
		newPointCloud->addDescriptor("stamps_sec", desc_sec);
		newPointCloud->addDescriptor("stamps_nsec", desc_nsec);

		cout << "Adding time" << endl;
		
	}

	ROS_INFO("Processing new point cloud");
	{
		timer t; // Print how long take the algo
		
		// Apply filters to incoming cloud, in scanner coordinates
		inputFilters.apply(*newPointCloud);
		
		ROS_INFO_STREAM("Input filters took " << t.elapsed() << " [s]");
	}
	
	string reason;
	// Initialize the transformation to identity if empty
 	if(!icp.hasMap())
 	{
		// we need to know the dimensionality of the point cloud to initialize properly
		publishLock.lock();
		TOdomToMap = PM::TransformationParameters::Identity(dimp1, dimp1);
		TOdomToMap(2,3) = mapElevation;
		publishLock.unlock();
	}

	cout << "TOdomToMap" << endl << TOdomToMap << endl;

	// Fetch transformation from scanner to odom
	// Note: we don't need to wait for transform. It is already called in transformListenerToEigenMatrix()
	PM::TransformationParameters TOdomToScanner;
	try
	{
		TOdomToScanner = PointMatcher_ros::eigenMatrixToDim<float>(
				PointMatcher_ros::transformListenerToEigenMatrix<float>(
				tfListener,
				scannerFrame,
				odomFrame,
				stamp
			), dimp1);
	}
	catch(tf::ExtrapolationException e)
	{
		ROS_ERROR_STREAM("Extrapolation Exception. stamp = "<< stamp << " now = " << ros::Time::now() << " delta = " << ros::Time::now() - stamp << endl << e.what() );
		return;
	}


	ROS_DEBUG_STREAM("TOdomToScanner(" << odomFrame<< " to " << scannerFrame << "):\n" << TOdomToScanner);
	ROS_DEBUG_STREAM("TOdomToMap(" << odomFrame<< " to " << mapFrame << "):\n" << TOdomToMap);
		
	const PM::TransformationParameters TscannerToMap = TOdomToMap * TOdomToScanner.inverse();
	ROS_DEBUG_STREAM("TscannerToMap (" << scannerFrame << " to " << mapFrame << "):\n" << TscannerToMap);
	
	// Ensure a minimum amount of point after filtering
	const int ptsCount = newPointCloud->features.cols();
	if(ptsCount < minReadingPointCount)
	{
		ROS_ERROR_STREAM("Not enough points in newPointCloud: only " << ptsCount << " pts.");
		return;
	}

	// Initialize the map if empty
 	if(!icp.hasMap())
 	{
		ROS_INFO_STREAM("Creating an initial map");
		mapCreationTime = stamp;
		setMap(updateMap(newPointCloud.release(), TscannerToMap, false));
		// we must not delete newPointCloud because we just stored it in the mapPointCloud
		return;
	}
	
	// Check dimension
	if (newPointCloud->features.rows() != icp.getInternalMap().features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << newPointCloud->features.rows()-1 << " while map is " << icp.getInternalMap().features.rows()-1);
		return;
	}
	
	try 
	{
		// Apply ICP
		PM::TransformationParameters Ticp;
		Ticp = icp(*newPointCloud, TscannerToMap);

		// ISER
		{
		// extract corrections
		PM::TransformationParameters Tdelta = Ticp * TscannerToMap.inverse();
		
		// remove roll and pitch
		Tdelta(2,0) = 0; 
		Tdelta(2,1) = 0; 
		Tdelta(2,2) = 1; 
		Tdelta(0,2) = 0; 
		Tdelta(1,2) = 0;
		Tdelta(2,3) = 0; //z
		Tdelta.block(0,0,3,3) = transformation->correctParameters(Tdelta.block(0,0,3,3));

		Ticp = Tdelta*TscannerToMap;

		}

		ROS_DEBUG_STREAM("Ticp:\n" << Ticp);
		
		// Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		ROS_INFO_STREAM("Overlap: " << estimatedOverlap);
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
			return;
		}
		
		// Compute tf
		publishStamp = stamp;
		publishLock.lock();
		TOdomToMap = Ticp * TOdomToScanner;
		// Publish tf
		if(publishMapTf == true)
		{
			tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TOdomToMap, mapFrame, odomFrame, stamp));
		}

		publishLock.unlock();
		processingNewCloud = false;
		
		ROS_DEBUG_STREAM("TOdomToMap:\n" << TOdomToMap);

		// Publish odometry
		if (odomPub.getNumSubscribers())
			odomPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(Ticp, mapFrame, stamp));
	
		// Publish error on odometry
		if (odomErrorPub.getNumSubscribers())
			odomErrorPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(TOdomToMap, mapFrame, stamp));

		

		// check if news points should be added to the map
		if (
			mapping &&
			((estimatedOverlap < maxOverlapToMerge) || (icp.getInternalMap().features.cols() < minMapPointCount)) &&
			#if BOOST_VERSION >= 104100
			(!mapBuildingInProgress)
			#else // BOOST_VERSION >= 104100
			true
			#endif // BOOST_VERSION >= 104100
		)
		{
			cout << "map Creation..." << endl;
			// make sure we process the last available map
			mapCreationTime = stamp;
			#if BOOST_VERSION >= 104100
			ROS_INFO("Adding new points to the map in background");
			mapBuildingTask = MapBuildingTask(boost::bind(&Mapper::updateMap, this, newPointCloud.release(), Ticp, true));
			mapBuildingFuture = mapBuildingTask.get_future();
			mapBuildingThread = boost::thread(boost::move(boost::ref(mapBuildingTask)));
			mapBuildingInProgress = true;
			#else // BOOST_VERSION >= 104100
			ROS_INFO("Adding new points to the map");
			setMap(updateMap( newPointCloud.release(), Ticp, true));
			#endif // BOOST_VERSION >= 104100
		}
	}
	catch (PM::ConvergenceError error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}
	
	//Statistics about time and real-time capability
	int realTimeRatio = 100*t.elapsed() / (stamp.toSec()-lastPoinCloudTime.toSec());
	ROS_INFO_STREAM("[TIME] Total ICP took: " << t.elapsed() << " [s]");
	if(realTimeRatio < 80)
		ROS_INFO_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");
	else
		ROS_WARN_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");

	lastPoinCloudTime = stamp;
}

void Mapper::processNewMapIfAvailable()
{
	#if BOOST_VERSION >= 104100
	if (mapBuildingInProgress && mapBuildingFuture.has_value())
	{
		ROS_INFO_STREAM("New map available");
		setMap(mapBuildingFuture.get());
		mapBuildingInProgress = false;
	}
	#endif // BOOST_VERSION >= 104100
}

void Mapper::setMap(DP* newPointCloud)
{
	// delete old map
	if (mapPointCloud)
		delete mapPointCloud;
	
	// set new map
	mapPointCloud = newPointCloud;
	cerr << "copying map to ICP" << endl;
	icp.setMap(*mapPointCloud);
	
	
	cerr << "publishing map" << endl;
	// Publish map point cloud
	// FIXME this crash when used without descriptor
	if (mapPub.getNumSubscribers())
		mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud, mapFrame, mapCreationTime));
}

Mapper::DP* Mapper::updateMap(DP* newPointCloud, const PM::TransformationParameters Ticp, bool updateExisting)
{
	timer t;

	// FIXME: those are parameters
	//const float priorStatic = 0.8; // ICRA 2014
	//const float priorDyn = 0.2; // ICRA 2014
	//const float priorStatic = 0.4; // ISER 2014
	//const float priorDyn = 0.6; // ISER 2014


	// Prepare empty field if not existing
	if(newPointCloud->descriptorExists("probabilityStatic") == false)
	{
		//newPointCloud->addDescriptor("probabilityStatic", PM::Matrix::Zero(1, newPointCloud->features.cols()));
		newPointCloud->addDescriptor("probabilityStatic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorStatic));
	}
	
	if(newPointCloud->descriptorExists("probabilityDynamic") == false)
	{
		//newPointCloud->addDescriptor("probabilityDynamic", PM::Matrix::Zero(1, newPointCloud->features.cols()));
		newPointCloud->addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorDyn));
	}
	
	if(newPointCloud->descriptorExists("dynamic_ratio") == false)
	{
		newPointCloud->addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, newPointCloud->features.cols()));
	}

	if (!updateExisting)
	{
		// FIXME: correct that, ugly
		cout << "Jumping map creation" << endl;
		*newPointCloud = transformation->compute(*newPointCloud, Ticp); 
		mapPostFilters.apply(*newPointCloud);
		return newPointCloud;
	}

	
	
	const int readPtsCount(newPointCloud->features.cols());
	const int mapPtsCount(mapPointCloud->features.cols());
	
	// Build a range image of the reading point cloud (local coordinates)
	PM::Matrix radius_reading = newPointCloud->features.topRows(3).colwise().norm();

	PM::Matrix angles_reading(2, readPtsCount); // 0=inclination, 1=azimuth

	// No atan in Eigen, so we are for to loop through it...
	for(int i=0; i<readPtsCount; i++)
	{
		const float ratio = newPointCloud->features(2,i)/radius_reading(0,i);
		//if(ratio < -1 || ratio > 1)
			//cout << "Error angle!" << endl;

		angles_reading(0,i) = acos(ratio);
		angles_reading(1,i) = atan2(newPointCloud->features(1,i), newPointCloud->features(0,i));
	}

	std::shared_ptr<NNS> featureNNS;
	featureNNS.reset( NNS::create(angles_reading));


	// Transform the global map in local coordinates
	DP mapLocalFrame = transformation->compute(*mapPointCloud, Ticp.inverse());

	// Remove points out of sensor range
	// FIXME: this is a parameter
	const float sensorMaxRange = 80.0;
	PM::Matrix globalId(1, mapPtsCount); 

	int mapCutPtsCount = 0;
	DP mapLocalFrameCut(mapLocalFrame.createSimilarEmpty());
	for (int i = 0; i < mapPtsCount; i++)
	{
		if (mapLocalFrame.features.col(i).head(3).norm() < sensorMaxRange)
		{
			mapLocalFrameCut.setColFrom(mapCutPtsCount, mapLocalFrame, i);
			globalId(0,mapCutPtsCount) = i;
			mapCutPtsCount++;
		}
	}

	mapLocalFrameCut.conservativeResize(mapCutPtsCount);
	//PM::Parameters params({{"maxDist", toParam(sensorMaxRange)}});
	//PM::DataPointsFilter* maxDistFilter = 
	//		PM::get().DataPointsFilterRegistrar.create("MaxDistDataPointsFilter", params);

	//mapLocalFrame = maxDistFilter->filter(mapLocalFrame);


	
	PM::Matrix radius_map = mapLocalFrameCut.features.topRows(3).colwise().norm();

	PM::Matrix angles_map(2, mapCutPtsCount); // 0=inclination, 1=azimuth

	// No atan in Eigen, so we are for to loop through it...
	for(int i=0; i<mapCutPtsCount; i++)
	{
		const float ratio = mapLocalFrameCut.features(2,i)/radius_map(0,i);
		//if(ratio < -1 || ratio > 1)
			//cout << "Error angle!" << endl;

		angles_map(0,i) = acos(ratio);

		angles_map(1,i) = atan2(mapLocalFrameCut.features(1,i), mapLocalFrameCut.features(0,i));
	}

	// Look for NN in spherical coordinates
	Matches::Dists dists(1,mapCutPtsCount);
	Matches::Ids ids(1,mapCutPtsCount);
	
	// FIXME: those are parameters
	// note: 0.08 rad is 5 deg
	//const float maxAngle = 0.04; // in rad (ICRA 14)
	//const float eps_a = 0.2; // ratio of distance (ICRA 14)
	//const float eps_d = 0.1; // in meters (ICRA 14)
	//const float maxAngle = 0.02; // in rad (ISER 14)
	//const float eps_a = 0.05; // ratio of distance (ISER 14)
	//const float eps_d = 0.02; // in meters (ISER 14)
	//const float alpha = 0.99;
	//const float beta = 0.99;

	
	
	featureNNS->knn(angles_map, ids, dists, 1, 0, NNS::ALLOW_SELF_MATCH, maxAngle);

	// Define views on descriptors
	//DP::View viewOn_normals_overlap = newPointCloud->getDescriptorViewByName("normals");
	//DP::View viewOn_obsDir_overlap = newPointCloud->getDescriptorViewByName("observationDirections");
	DP::View viewOn_Msec_overlap = newPointCloud->getDescriptorViewByName("stamps_Msec");
	DP::View viewOn_sec_overlap = newPointCloud->getDescriptorViewByName("stamps_sec");
	DP::View viewOn_nsec_overlap = newPointCloud->getDescriptorViewByName("stamps_nsec");

	DP::View viewOnProbabilityStatic = mapPointCloud->getDescriptorViewByName("probabilityStatic");
	DP::View viewOnProbabilityDynamic = mapPointCloud->getDescriptorViewByName("probabilityDynamic");
	DP::View viewOnDynamicRatio = mapPointCloud->getDescriptorViewByName("dynamic_ratio");
	
	DP::View viewOn_normals_map = mapPointCloud->getDescriptorViewByName("normals");
	DP::View viewOn_Msec_map = mapPointCloud->getDescriptorViewByName("stamps_Msec");
	DP::View viewOn_sec_map = mapPointCloud->getDescriptorViewByName("stamps_sec");
	DP::View viewOn_nsec_map = mapPointCloud->getDescriptorViewByName("stamps_nsec");
	
	

	viewOnDynamicRatio = PM::Matrix::Zero(1, mapPtsCount);
	for(int i=0; i < mapCutPtsCount; i++)
	{
		if(dists(i) != numeric_limits<float>::infinity())
		{
			const int readId = ids(0,i);
			const int mapId = globalId(0,i);
			
			// in local coordinates
			const Eigen::Vector3f readPt = newPointCloud->features.col(readId).head(3);
			const Eigen::Vector3f mapPt = mapLocalFrameCut.features.col(i).head(3);
			const Eigen::Vector3f mapPt_n = mapPt.normalized();
			const float delta = (readPt - mapPt).norm();
			const float d_max = eps_a * readPt.norm();

			//const double timeMap = viewOn_Msec_map(0,mapId)*10e5 + viewOn_sec_map(0,mapId) + viewOn_nsec_map(0,mapId)*10e-10;
			//const double timeRead = viewOn_Msec_overlap(0,readId)*10e5 + viewOn_sec_overlap(0,readId) + viewOn_nsec_overlap(0,readId)*10e-10;
			//const double deltaTime = timeRead - timeMap;

			const Eigen::Vector3f normal_map = viewOn_normals_map.col(mapId);
			//const Eigen::Vector3f normal_read = viewOn_normals_overlap.col(readId);
			//const Eigen::Vector3f obsDir = viewOn_obsDir_overlap.col(readId);
			//const float viewAngle = acos(normal_map.normalized().dot(obsDir.normalized()));
			//const float normalAngle = acos(normal_map.normalized().dot(normal_read.normalized()));
			
			// Weight for dynamic elements
			const float w_v = eps + (1 - eps)*fabs(normal_map.dot(mapPt_n));
			//const float w_d1 = 1 + eps - acos(readPt.normalized().dot(mapPt_n))/maxAngle;
			const float w_d1 =  eps + (1 - eps)*(1 - sqrt(dists(i))/maxAngle);
			
			
			const float offset = delta - eps_d;
			float w_d2 = 1;
			if(delta < eps_d || mapPt.norm() > readPt.norm())
			{
				w_d2 = eps;
			}
			else 
			{
				if (offset < d_max)
				{
					w_d2 = eps + (1 - eps )*offset/d_max;
				}
			}

			float w_p2 = eps;
			if(delta < eps_d)
			{
				w_p2 = 1;
			}
			else
			{
				if(offset < d_max)
				{
					w_p2 = eps + (1 - eps)*(1 - offset/d_max);
				}
			}

			//cerr << "readPt.norm(): "<< readPt.norm()  << "mapPt.norm(): "<< mapPt.norm() << ", w_p2: " << w_p2 << ", w_d2: " << w_d2 << endl;
		

			// We don't update point behind the reading
			if((readPt.norm() + eps_d + d_max) >= mapPt.norm())
			{
				const float lastDyn = viewOnProbabilityDynamic(0,mapId);
				const float lastStatic = viewOnProbabilityStatic(0, mapId);

				const float c1 = (1 - (w_v*(1 - w_d1)));
				const float c2 = w_v*(1 - w_d1);
				

				//viewOnProbabilityDynamic(0,mapId) += (w_v + w_d2) * w_d1/2;
				//viewOnProbabilityDynamic(0,mapId) += (w_v * w_d2);
				
				//viewOnProbabilityStatic(0, mapId) += (w_p2) * w_d1;
				//viewOnProbabilityStatic(0, mapId) += w_p2;

				// FIXME: this is a parameter
				//const float maxDyn = 0.9; // ICRA 14
				//const float maxDyn = 0.98; // ISER 14

				//Lock dynamic point to stay dynamic under a threshold
				if(lastDyn < maxDyn)
				{
					viewOnProbabilityDynamic(0,mapId) = c1*lastDyn + c2*w_d2*((1 - alpha)*lastStatic + beta*lastDyn);
					viewOnProbabilityStatic(0, mapId) = c1*lastStatic + c2*w_p2*(alpha*lastStatic + (1 - beta)*lastDyn);
				}
				else
				{
					viewOnProbabilityStatic(0,mapId) = eps;
					viewOnProbabilityDynamic(0,mapId) = 1-eps;
				}
				
				
				
				// normalization
				const float sumZ = viewOnProbabilityDynamic(0,mapId) + viewOnProbabilityStatic(0, mapId);
				assert(sumZ >= eps);	
				
				viewOnProbabilityDynamic(0,mapId) /= sumZ;
				viewOnProbabilityStatic(0,mapId) /= sumZ;
				
				//viewOnDynamicRatio(0,mapId) =viewOnProbabilityDynamic(0, mapId);
				viewOnDynamicRatio(0,mapId) = w_d2;

				//viewOnDynamicRatio(0,mapId) =	w_d2;

				// Refresh time
				viewOn_Msec_map(0,mapId) = viewOn_Msec_overlap(0,readId);	
				viewOn_sec_map(0,mapId) = viewOn_sec_overlap(0,readId);	
				viewOn_nsec_map(0,mapId) = viewOn_nsec_overlap(0,readId);	
			}


		}
	}




	// Correct new points using ICP result
	//*newPointCloud = transformation->compute(*newPointCloud, Ticp); 

	// Generate temporary map for density computation
	//DP tmp_map = (*mapPointCloud); // FIXME: this should be on mapLocalFrameCut
	DP tmp_map = mapLocalFrameCut; // FIXME: this should be on mapLocalFrameCut
	tmp_map.concatenate(*newPointCloud);

	// Compute density
	//std::shared_ptr<NNS> featureNNS;

	cout << "build first kdtree with " << tmp_map.features.cols() << endl;
	// build and populate NNS
	featureNNS.reset( NNS::create(tmp_map.features, tmp_map.features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
	
	PM::Matches matches_overlap(
		Matches::Dists(1, readPtsCount),
		Matches::Ids(1, readPtsCount)
	);
	
	featureNNS->knn(newPointCloud->features, matches_overlap.ids, matches_overlap.dists, 1, 0);
	
	
	DP overlap(newPointCloud->createSimilarEmpty());
	DP no_overlap(newPointCloud->createSimilarEmpty());
	
	

	// FIXME: this is a parameter
  //const float maxDist = pow(0.3, 2); // ICRA 2014
	//const float maxDist = pow(0.1, 2); // ISER 2014
	//const float maxDist = pow(0.1, 2);

	int ptsOut = 0;
	int ptsIn = 0;
	for (int i = 0; i < readPtsCount; ++i)
	{
		if (matches_overlap.dists(i) > maxDistNewPoint)
		{
			no_overlap.setColFrom(ptsOut, *newPointCloud, i);
			ptsOut++;
		}
		else
		{
			overlap.setColFrom(ptsIn, *newPointCloud, i);
			ptsIn++;
		}
	}

	no_overlap.conservativeResize(ptsOut);
	overlap.conservativeResize(ptsIn);

	cout << "ptsOut=" << ptsOut << ", ptsIn=" << ptsIn << endl;

	// Publish outliers
	//if (outlierPub.getNumSubscribers())
	//{
		//outlierPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(no_overlap, mapFrame, mapCreationTime));
	//}

	// Initialize descriptors
	//no_overlap.addDescriptor("probabilityStatic", PM::Matrix::Zero(1, no_overlap.features.cols()));
	no_overlap.addDescriptor("probabilityStatic", PM::Matrix::Constant(1, no_overlap.features.cols(), priorStatic));
	//no_overlap.addDescriptor("probabilityDynamic", PM::Matrix::Zero(1, no_overlap.features.cols()));
	no_overlap.addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, no_overlap.features.cols(), priorDyn));

	no_overlap.addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, no_overlap.features.cols()));

	// shrink the newPointCloud to the new information
	*newPointCloud = no_overlap;
	

	
	// Correct new points using ICP result
	*newPointCloud = transformation->compute(*newPointCloud, Ticp);
	
	
	
	
	// Merge point clouds to map
	newPointCloud->concatenate(*mapPointCloud);
	mapPostFilters.apply(*newPointCloud);

	cout << "... end map creation" << endl;
	ROS_INFO_STREAM("[TIME] New map available (" << newPointCloud->features.cols() << " pts), update took " << t.elapsed() << " [s]");
	
	return newPointCloud;
}

void Mapper::waitForMapBuildingCompleted()
{
	#if BOOST_VERSION >= 104100
	if (mapBuildingInProgress)
	{
		// we wait for now, in future we should kill it
		mapBuildingFuture.wait();
		mapBuildingInProgress = false;
	}
	#endif // BOOST_VERSION >= 104100
}

void Mapper::publishLoop(double publishPeriod)
{
	if(publishPeriod == 0)
		return;
	ros::Rate r(1.0 / publishPeriod);
	while(ros::ok())
	{
		publishTransform();
		r.sleep();
	}
}

void Mapper::publishTransform()
{
	if(processingNewCloud == false && publishMapTf == true)
	{
		publishLock.lock();
		// Note: we use now as timestamp to refresh the tf and avoid other buffer to be empty
		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TOdomToMap, mapFrame, odomFrame, ros::Time::now()));
		publishLock.unlock();
	}
}

bool Mapper::getPointMap(map_msgs::GetPointMap::Request &req, map_msgs::GetPointMap::Response &res)
{
	if (!mapPointCloud)
		return false;
	
	// FIXME: do we need a mutex here?
	res.map = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud, mapFrame, ros::Time::now());
	return true;
}

bool Mapper::saveMap(map_msgs::SaveMap::Request &req, map_msgs::SaveMap::Response &res)
{
	if (!mapPointCloud)
		return false;
	
	try
	{
		mapPointCloud->save(req.filename.data);
	}
	catch (const std::runtime_error& e)
	{
		ROS_ERROR_STREAM("Unable to save: " << e.what());
		return false;
	}
	
	ROS_INFO_STREAM("Map saved at " <<  req.filename.data << " with " << mapPointCloud->features.cols() << " points.");
	return true;
}

bool Mapper::loadMap(ethzasl_icp_mapper::LoadMap::Request &req, ethzasl_icp_mapper::LoadMap::Response &res)
{
	waitForMapBuildingCompleted();
	
	DP* cloud(new DP(DP::load(req.filename.data)));

	const int dim = cloud->features.rows();
	const int nbPts = cloud->features.cols();
	ROS_INFO_STREAM("Loading " << dim-1 << "D point cloud (" << req.filename.data << ") with " << nbPts << " points.");

	publishLock.lock();
	TOdomToMap = PM::TransformationParameters::Identity(dim,dim);
	
	//ISER
	TOdomToMap(2,3) = mapElevation;
	publishLock.unlock();

	setMap(cloud);
	
	return true;
}

bool Mapper::reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	waitForMapBuildingCompleted();
	
	// note: no need for locking as we do ros::spin(), to update if we go for multi-threading
	publishLock.lock();
	TOdomToMap = PM::TransformationParameters::Identity(4,4);
	publishLock.unlock();

	icp.clearMap();
	
	return true;
}

bool Mapper::correctPose(ethzasl_icp_mapper::CorrectPose::Request &req, ethzasl_icp_mapper::CorrectPose::Response &res)
{
	publishLock.lock();
	TOdomToMap = PointMatcher_ros::odomMsgToEigenMatrix<float>(req.odom);
	
	//ISER
	{
	// remove roll and pitch
	TOdomToMap(2,0) = 0; 
	TOdomToMap(2,1) = 0; 
	TOdomToMap(2,2) = 1; 
	TOdomToMap(0,2) = 0; 
	TOdomToMap(1,2) = 0;
	TOdomToMap(2,3) = mapElevation; //z
	}

	tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TOdomToMap, mapFrame, odomFrame, ros::Time::now()));
	publishLock.unlock();

	return true;
}

bool Mapper::setMode(ethzasl_icp_mapper::SetMode::Request &req, ethzasl_icp_mapper::SetMode::Response &res)
{
	// Impossible states
	if(req.localize == false && req.map == true)
		return false;

	localizing = req.localize;
	mapping = req.map;
	
	return true;
}

bool Mapper::getMode(ethzasl_icp_mapper::GetMode::Request &req, ethzasl_icp_mapper::GetMode::Response &res)
{
	res.localize = localizing;
	res.map = mapping;
	return true;
}



bool Mapper::getBoundedMap(ethzasl_icp_mapper::GetBoundedMap::Request &req, ethzasl_icp_mapper::GetBoundedMap::Response &res)
{
	if (!mapPointCloud)
		return false;

	const float max_x = req.topRightCorner.x;
	const float max_y = req.topRightCorner.y;
	const float max_z = req.topRightCorner.z;

	const float min_x = req.bottomLeftCorner.x;
	const float min_y = req.bottomLeftCorner.y;
	const float min_z = req.bottomLeftCorner.z;

	cerr << "min [" << min_x << ", " << min_y << ", " << min_z << "] " << endl;
	cerr << "max [" << max_x << ", " << max_y << ", " << max_z << "] " << endl;



	tf::StampedTransform stampedTr;
	
	Eigen::Affine3d eigenTr;
	tf::poseMsgToEigen(req.mapCenter, eigenTr);
	Eigen::MatrixXf T = eigenTr.matrix().inverse().cast<float>();
	//const Eigen::MatrixXf T = eigenTr.matrix().cast<float>();

	cerr << "T:" << endl << T << endl;
	T = transformation->correctParameters(T);

		
	// FIXME: do we need a mutex here?
	const DP centeredPointCloud = transformation->compute(*mapPointCloud, T); 
	DP cutPointCloud = centeredPointCloud.createSimilarEmpty();

	cerr << centeredPointCloud.features.topLeftCorner(3, 10) << endl;
	cerr << T << endl;

	int newPtCount = 0;
	for(int i=0; i < centeredPointCloud.features.cols(); i++)
	{
		const float x = centeredPointCloud.features(0,i);
		const float y = centeredPointCloud.features(1,i);
		const float z = centeredPointCloud.features(2,i);
		
		if(x < max_x && x > min_x &&
			 y < max_y && y > min_y &&
		   z < max_z && z > min_z 	)
		{
			cutPointCloud.setColFrom(newPtCount, centeredPointCloud, i);
			newPtCount++;	
		}
	}

	cerr << "Extract " << newPtCount << " points from the map" << endl;
	
	cutPointCloud.conservativeResize(newPtCount);
	cutPointCloud = transformation->compute(cutPointCloud, T.inverse()); 

	
	// Send the resulting point cloud in ROS format
	res.boundedMap = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(cutPointCloud, mapFrame, ros::Time::now());
	return true;
}

// Main function supporting the Mapper class
int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapper");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	Mapper mapper(n, pn);
	ros::spin();
	
	return 0;
}
