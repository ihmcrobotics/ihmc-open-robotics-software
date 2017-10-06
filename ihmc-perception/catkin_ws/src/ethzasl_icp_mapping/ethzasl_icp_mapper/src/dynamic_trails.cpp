#include <fstream>

#include <boost/version.hpp>
#include <deque>


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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

using namespace std;
using namespace PointMatcherSupport;
using namespace visualization_msgs;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

class DynamicTrails
{
	typedef PM::TransformationParameters TP;
	typedef PM::Matches Matches;
	
	typedef typename Nabo::NearestNeighbourSearch<float> NNS;
	typedef typename NNS::SearchType NNSearchType;

	ros::NodeHandle& n;
	ros::NodeHandle& pn;

	//message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
	//message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

	//typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
	//message_filters::Synchronizer<MySyncPolicy> sync;

	ros::Subscriber scanSub;
	ros::Subscriber cloudSub;
	ros::Subscriber mapSub;
	ros::Publisher lastPub;
	ros::Publisher dynPub;
	ros::Publisher lastProjectedPub;
	ros::Publisher dynProjectedPub;
	ros::Publisher markerPub;

	PM::DataPointsFilters inputFilters;
	unique_ptr<PM::Transformation> transformation;
	shared_ptr<DP> lastPointCloud;
	shared_ptr<DP> trailPointCloud;
	PM::DataPoints globalMap;
	
	std::shared_ptr<NNS> globalNNS;
	std::shared_ptr<NNS> lastNNS;

	boost::mutex usingGlobalMap;
	
	std::deque<shared_ptr<DP>> priors;

	// tf
	tf::TransformListener tfListener;

	boost::mutex publishLock;
	
	const float eps;
	ros::Time lastInputTime;

	// Parameters
	const string p_mapFrame;
	const float p_minDynamicRatio;
	const float p_normalSpaceFactor;
	const float p_maxVelocity;
	const float p_inputRate;
	const int p_windowSize;
	const string p_trailFilepName;
	int trailCount;
	
public:
	DynamicTrails(ros::NodeHandle& n, ros::NodeHandle& pn);
	~DynamicTrails();
	
protected:
	//void gotCloud(const sensor_msgs::PointCloud2ConstPtr& cloudMsgIn, const nav_msgs::OdometryConstPtr& odom);
	void gotCloud(const sensor_msgs::PointCloud2ConstPtr& cloudMsgIn);
	void gotMap(const sensor_msgs::PointCloud2& cloudMsgIn);
	void processCloud(DP cloud, const TP TScannerToMap);
	PM::DataPoints extractDynamicPoints(const PM::DataPoints input, const float minDynamicRatio);
	void computeVelocity(shared_ptr<DP> reference, shared_ptr<NNS> ref_tree, shared_ptr<DP> reading, shared_ptr<NNS> read_tree, const float maxRadius, const int knn, ros::Publisher debugPub);
	float angleWeight(const Eigen::Vector3f &normal, const Eigen::Vector3f &velocity);
	
	void averageWithReference(shared_ptr<DP> reference, shared_ptr<DP> reading, const float dir);

};

DynamicTrails::DynamicTrails(ros::NodeHandle& n, ros::NodeHandle& pn):
	n(n),
	pn(pn),
	//cloud_sub(n, "cloud_in", 1),
	//odom_sub(n, "icp_odom", 1),
	//sync(MySyncPolicy(10), cloud_sub, odom_sub),
	transformation(PM::get().REG(Transformation).create("RigidTransformation")),
	lastPointCloud(0),
	trailPointCloud(0),
  tfListener(ros::Duration(30)),
	eps(0.0001),
	p_mapFrame(getParam<string>("map_frame", "map")),
	p_minDynamicRatio(getParam<double>("min_dynamic_ratio", 0.75)),
	p_normalSpaceFactor(getParam<double>("normal_space_factor", 1.0)),
	p_maxVelocity(getParam<double>("max_velocity", 12.0)),
	p_inputRate(getParam<double>("input_rate", 1.5)),
	p_windowSize(getParam<int>("window_size", 3)),
	p_trailFilepName(getParam<string>("trail_filename", "trailMap.vtk")),
	trailCount(0)
{
	
	// memory for last velocities
	//priors.resize(p_windowSize);


	// load configs
	string configFileName;

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

	
	// topics and services initialization
	//sync.registerCallback(boost::bind(&DynamicTrails::gotCloud, this, _1, _2));
	
	cloudSub = n.subscribe("cloud_in", 10, &DynamicTrails::gotCloud, this);
	mapSub = n.subscribe("point_map", 1, &DynamicTrails::gotMap, this);
	lastPub = n.advertise<sensor_msgs::PointCloud2>("last_dynamic_points", 2, true);
	dynPub = n.advertise<sensor_msgs::PointCloud2>("current_dynamic_points", 2, true);
	lastProjectedPub= n.advertise<sensor_msgs::PointCloud2>("last_projected_points", 2, true);
	dynProjectedPub= n.advertise<sensor_msgs::PointCloud2>("dyn_projected_points", 2, true);
	markerPub = n.advertise<visualization_msgs::Marker>("dynamic_trail", 1);
		
}

DynamicTrails::~DynamicTrails()
{
	// save point cloud
	
	if (trailPointCloud)
	{
		trailPointCloud->save(p_trailFilepName);
	}

}

// Callback
void DynamicTrails::gotMap(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	cout << "Map received" << endl;
	usingGlobalMap.lock();
	// Convert pointcloud2 to DataPoint
	globalMap = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn);
	
	// Build a kd-tree
	globalNNS.reset( NNS::create(globalMap.features, globalMap.features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
	usingGlobalMap.unlock();
}



void DynamicTrails::gotCloud(const sensor_msgs::PointCloud2ConstPtr& cloudMsgIn)
{
	// IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)

	const ros::Time currentTime = ros::Time::now();
	const float currentRate = 1.0/(currentTime - lastInputTime).toSec();
	
	if(currentRate < p_inputRate )
	{
		lastInputTime = currentTime;

		// Convert pointcloud2 to DataPoint
		DP cloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*cloudMsgIn));
		
		// Convert odometry msg to TransformationParameters
		//TP TScannerToMap = 	PointMatcher_ros::odomMsgToEigenMatrix<float>(*odom);
		TP TScannerToMap;

		try
		{
			tfListener.waitForTransform(p_mapFrame, cloudMsgIn->header.frame_id, cloudMsgIn->header.stamp, ros::Duration(0.5));
			TScannerToMap = PointMatcher_ros::transformListenerToEigenMatrix<float>(
					tfListener,
					p_mapFrame,
					cloudMsgIn->header.frame_id,
					cloudMsgIn->header.stamp
				);
			processCloud(cloud, TScannerToMap);
		}
		catch(tf::ExtrapolationException e)
		{
			ROS_ERROR_STREAM("Extrapolation Exception. stamp = "<< cloudMsgIn->header.stamp << " now = " << ros::Time::now() << " delta = " << ros::Time::now() - cloudMsgIn->header.stamp);
			return;
		}
	
	}

}

// Point cloud processing
void DynamicTrails::processCloud(DP inputPointCloud, const TP TScannerToMap)
{

	ROS_INFO("Processing new point cloud");
	
	timer t; // Print how long take the algo
		

		
	ROS_INFO_STREAM("Input filters took " << t.elapsed() << " [s]");
	
	// Correct new points using ICP result and move them in their own frame
	inputPointCloud = transformation->compute(inputPointCloud, TScannerToMap); 

	cerr << "inputPointCloud.features.cols() " << inputPointCloud.features.cols() << endl;
	// filter to keep only dynamic points
	shared_ptr<DP> dynamicPointCloud( new DP(extractDynamicPoints(inputPointCloud, p_minDynamicRatio)));

	
	
	// Apply filters to incoming cloud, in global coordinates!
	inputFilters.apply(*dynamicPointCloud);
	
	const int dynamicPtsCount = dynamicPointCloud->features.cols();

	cerr << "dynamicPointCloud->features.cols() " << dynamicPtsCount << endl;
	
	// Error verification
	if(dynamicPtsCount == 0)
	{
		ROS_WARN_STREAM("No dynamic points could be found");
		return;
	}
		
	//PM::Matrix unitVec(3,dynamicPtsCount);
	//unitVec.row(0) = PM::Matrix::Ones(1,dynamicPtsCount);
	//unitVec.bottomRows(2) = PM::Matrix::Zero(2,dynamicPtsCount);


	dynamicPointCloud->addDescriptor("velocity_dir", PM::Matrix::Zero(3,dynamicPtsCount));
	dynamicPointCloud->addDescriptor("velocity_mag", PM::Matrix::Zero(1,dynamicPtsCount));
	dynamicPointCloud->addDescriptor("velocity_dt", PM::Matrix::Zero(1,dynamicPtsCount));


	// fill the lastPointCloud for the firs time
	if(!lastPointCloud)
	{
		lastPointCloud = dynamicPointCloud;
		cerr << lastPointCloud->features.cols() << endl;


		DP::View viewOn_normals_last = lastPointCloud->getDescriptorViewByName("normals");

		// Build the kd-tree of the last points
		PM::Matrix lastEuclNormal(6, dynamicPtsCount);
		lastEuclNormal.topRows(3) = lastPointCloud->features.topRows(3);
		lastEuclNormal.bottomRows(3) = viewOn_normals_last*p_normalSpaceFactor;

		//lastNNS.reset( NNS::create(lastEuclNormal, 6, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
		lastNNS.reset( NNS::create(lastPointCloud->features, lastPointCloud->features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));

		ROS_INFO_STREAM("Initializing last point cloud");

		return;
	}

	const int lastPtsCount = lastPointCloud->features.cols();
	ROS_INFO_STREAM("lastPtsCount: " << lastPtsCount);

	// Error check on dimensions
	if (dynamicPointCloud->features.rows() != lastPointCloud->features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << dynamicPointCloud->features.rows()-1 << " while lastPointCloud is " << lastPointCloud->features.rows()-1);
		return;
	}

	

	// Copy the last speed direction for temporal smoothing
	PM::Matrix priorOn_velocityDir = lastPointCloud->getDescriptorViewByName("velocity_dir");
	PM::Matrix priorOn_velocityDt = lastPointCloud->getDescriptorViewByName("velocity_dt");
	
	DP::View v_normals_input = dynamicPointCloud->getDescriptorViewByName("normals");
	PM::Matrix inputEuclNormal(6, dynamicPtsCount);
	inputEuclNormal.topRows(3) = dynamicPointCloud->features.topRows(3);
	inputEuclNormal.bottomRows(3) = v_normals_input*p_normalSpaceFactor;
	

	// Build kd-tree of the input points
	std::shared_ptr<NNS> inputNNS;
	//inputNNS.reset( NNS::create(inputEuclNormal, 6, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
	inputNNS.reset( NNS::create(dynamicPointCloud->features, 3, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));


	// FIXME: those are parameters
	const float maxClusterDistance = 0.75;
	const int nbIter = 5;
	for(int i=0; i<nbIter; i++)
	{
		computeVelocity(lastPointCloud, lastNNS, dynamicPointCloud, inputNNS, maxClusterDistance, nbIter-i, dynProjectedPub);
		computeVelocity(dynamicPointCloud, inputNNS, lastPointCloud, lastNNS, maxClusterDistance, nbIter-i, lastProjectedPub);

		averageWithReference(lastPointCloud, dynamicPointCloud, 1);
		averageWithReference(dynamicPointCloud, lastPointCloud, 1);
	}
	

	DP::View v_matchId_dyn = dynamicPointCloud->getDescriptorViewByName("match_id");
	//DP::View v_velocityMag_dyn = dynamicPointCloud->getDescriptorViewByName("velocity_mag");
	DP::View v_velocityDir_dyn = dynamicPointCloud->getDescriptorViewByName("velocity_dir");
	DP::View v_velocityDt_dyn = dynamicPointCloud->getDescriptorViewByName("velocity_dt");
	DP::View v_normal_dyn = dynamicPointCloud->getDescriptorViewByName("normals");

	//DP::View v_velocityMag_last = lastPointCloud->getDescriptorViewByName("velocity_mag");
	DP::View v_velocityDir_last = lastPointCloud->getDescriptorViewByName("velocity_dir");
	DP::View v_velocityDt_last = lastPointCloud->getDescriptorViewByName("velocity_dt");
	
	// Project into the future
	v_velocityDir_dyn *= -1;
	
	// Markers
	visualization_msgs::Marker marker;

	marker.header.frame_id = p_mapFrame;
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	//marker.id = trailCount;
	trailCount++;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	
	marker.scale.x = 0.01;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	//marker.color.a = 0.20;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	DP currenttTrails = dynamicPointCloud->createSimilarEmpty();
	int currentTrailPtCount = 0;

	// Filter outliers and build markers
	for(int i=0; i<dynamicPtsCount; i++)
	{
		const int id = v_matchId_dyn(0,i);

		const Eigen::Vector3f pt = dynamicPointCloud->features.col(i).head(3);
		const Eigen::Vector3f pt_last = lastPointCloud->features.col(id).head(3);
		
		const Eigen::Vector3f normal = v_normal_dyn.col(i);
		const float w_ang = angleWeight(normal, v_velocityDir_dyn.col(i));
		
		const Eigen::Vector3f lastVelDir = priorOn_velocityDir.col(id);
		const Eigen::Vector3f currentVelDir = v_velocityDir_dyn.col(i);
		const float w_diffAngle =  1 - acos(lastVelDir.normalized().dot(currentVelDir.normalized()))/M_PI;
		
		
		int lastID = id;

		// FIXME: finish here
		//for(int i = 0; i < priors.size(); i++)
		//{
		//	const shared_ptr<DP> lastPrior = priors[i];
		//	cerr << lastPrior << endl;
		//	cerr << "lastID " << lastID << endl;
		//	cerr << "last size " << lastPrior->features.cols() << endl;
		//	DP::View match_id = priors[i]->getDescriptorViewByName("match_id");

		//	lastID = match_id(0, lastID);
		//	
		//}

		//cerr << "----------" << endl;
		
		const float lastDt = priorOn_velocityDt(0,id);
		const float delta = (pt_last - pt).norm();
		const float delta_projected = (pt_last + lastVelDir*lastDt - pt).norm();
		const float w_proj = delta_projected/delta;


		const float mag = currentVelDir.norm();
		//if(w_ang > 0.75  && mag > eps && mag < p_maxVelocity)
		if(mag > eps && mag < p_maxVelocity)
		{
			if(w_diffAngle > 0.5 && w_proj < 0.5)
			{
				// Temporal smoothing

				v_velocityDir_dyn.col(i) = (lastVelDir + currentVelDir)/2;

				geometry_msgs::Point p;
				p.x = dynamicPointCloud->features(0,i);
				p.y = dynamicPointCloud->features(1,i);
				p.z = dynamicPointCloud->features(2,i);
				marker.points.push_back(p);
				
				p.x += v_velocityDir_dyn(0,i);
				p.y += v_velocityDir_dyn(1,i);
				p.z += v_velocityDir_dyn(2,i);

				marker.points.push_back(p);

				currenttTrails.setColFrom(currentTrailPtCount, *dynamicPointCloud, i);
				currentTrailPtCount++;
			}
		}
		else
		{
			v_velocityDir_dyn.col(i) = Eigen::Vector3f::Zero();
			v_velocityDt_dyn(0,i) = 0;
		}
		
	}

	currenttTrails.conservativeResize(currentTrailPtCount);

	if(currentTrailPtCount > 0)
	{
		if(trailPointCloud == false)
		{
			trailPointCloud.reset(new DP(currenttTrails));
		}
		else
		{
			trailPointCloud->concatenate(currenttTrails);
		}
	}

	markerPub.publish(marker);

	dynPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*dynamicPointCloud.get(), p_mapFrame, ros::Time::now()));
	lastPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*lastPointCloud.get(), p_mapFrame, ros::Time::now()));


	lastPointCloud = dynamicPointCloud;
	lastNNS = inputNNS;


	if(priors.size() >= p_windowSize)
	{
		priors.pop_back();
	}

	priors.push_front(dynamicPointCloud);


	ROS_INFO_STREAM("Total trail computation: " << t.elapsed() << " [s]");

	//ros::Rate r(2);
	//r.sleep();
}


PM::DataPoints DynamicTrails::extractDynamicPoints (const PM::DataPoints input, const float minDynamicRatio)
{
	const int inputPtsCount(input.features.cols());

	DP output(input.createSimilarEmpty());

	const int knn = 1;
	Matches::Dists dists_input(knn, inputPtsCount);
	Matches::Ids ids_input(knn, inputPtsCount);
	
	usingGlobalMap.lock();
	// FIXME: this is a parameter
	const float maxGlobalDist = 0.5;
	globalNNS->knn(input.features, ids_input, dists_input, knn, 0, 0, maxGlobalDist);

	if(globalMap.descriptorExists("probabilityStatic") == false ||
			globalMap.descriptorExists("probabilityDynamic") == false ||
			globalMap.descriptorExists("dynamic_ratio") == false
		)
	{
		ROS_WARN_STREAM("Map descriptors not found");
		usingGlobalMap.unlock();
		return output;
	}

	//DP::View viewOnDynamicRatio = globalMap.getDescriptorViewByName("dynamic_ratio");
	DP::View viewOnProbabilityStatic = globalMap.getDescriptorViewByName("probabilityStatic");
	DP::View viewOnProbabilityDynamic = globalMap.getDescriptorViewByName("probabilityDynamic");

	int dynamicPtsCount=0;
	for(int i=0; i<inputPtsCount; i++)
	{
		float sumObs = 0;
		float dist = numeric_limits<float>::infinity();
		float dynRatio = 0;

		for(int k=0; k<knn; k++)
		{
			const int mapId = ids_input(k,i);
			sumObs = max(sumObs, viewOnProbabilityDynamic(0,mapId) + viewOnProbabilityStatic(0,mapId));
			dist = min(dist, dists_input(k,i));
			dynRatio = max(dynRatio, viewOnProbabilityDynamic(0, mapId));
		}

		{
			//if(sumObs == 0 || dynRatio > minDynamicRatio || dist == numeric_limits<float>::infinity())
			if(dynRatio > minDynamicRatio || dist == numeric_limits<float>::infinity())
			{
				output.setColFrom(dynamicPtsCount, input, i);
				dynamicPtsCount++;
			}
		}
	}

	usingGlobalMap.unlock();

	output.conservativeResize(dynamicPtsCount);
	return output;

}

void DynamicTrails::computeVelocity(shared_ptr<DP> reference, shared_ptr<NNS> ref_tree, shared_ptr<DP> reading, shared_ptr<NNS> read_tree, const float maxRadius, const int knn, ros::Publisher debugPub)
{

	const int readPtsCount = reading->features.cols();
	const int refPtsCount = reference->features.cols();

	// Prepare useful views on descriptors
	DP::View v_velocityDir_read = reading->getDescriptorViewByName("velocity_dir");
	DP::View v_velocityDt_read = reading->getDescriptorViewByName("velocity_dt");
	DP::View v_normal_read = reading->getDescriptorViewByName("normals");

	DP::View v_Msec_read = reading->getDescriptorViewByName("stamps_Msec");
	DP::View v_sec_read = reading->getDescriptorViewByName("stamps_sec");
	DP::View v_nsec_read = reading->getDescriptorViewByName("stamps_nsec");
	
	// Ensure that we don't modify the reference
	PM::Matrix velocityDir_ref = PM::Matrix::Zero(3,refPtsCount);
	PM::Matrix velocityDt_ref = PM::Matrix::Zero(1,refPtsCount);

	DP::View v_Msec_ref = reference->getDescriptorViewByName("stamps_Msec");
	DP::View v_sec_ref = reference->getDescriptorViewByName("stamps_sec");
	DP::View v_nsec_ref = reference->getDescriptorViewByName("stamps_nsec");
	

	// Move points based on velocity
	PM::Matrix moved_read(3, readPtsCount);
	//PM::Matrix moved_read(6, readPtsCount);

	const PM::Matrix static_read = reading->features.topRows(3);
	//PM::Matrix static_read(6, readPtsCount);
	//static_read.topRows(3) = reading->features.topRows(3);
	//static_read.bottomRows(3) = v_normal_read;
	
	//if (knn == 1)
	//	abort();

	for(int i=0; i<readPtsCount; i++)
	{
		moved_read.col(i)= reading->features.col(i).head(3) + v_velocityDir_read.col(i)*v_velocityDt_read(0,i);
		//moved_read.col(i)= reading->features.col(i).head(3) + v_velocityDir_read.col(i);
		
		//moved_read.topRows(3).col(i)= reading->features.col(i).head(3) + v_velocityDir_read.col(i)*v_velocityMag_read(0,i);
		//moved_read.bottomRows(3).col(i) = v_normal_read.col(i);
		
		
		assert(!isnan(moved_read(0,i)));
		assert(!isnan(moved_read(1,i)));
		assert(!isnan(moved_read(2,i)));
		assert(!isinf(moved_read(0,i)));
		assert(!isinf(moved_read(1,i)));
		assert(!isinf(moved_read(2,i)));
	}

	//------------------
	// Search for nearest points
	Matches::Dists dists(knn, readPtsCount);
	Matches::Ids ids(knn, readPtsCount);

	ref_tree->knn(moved_read, ids, dists, knn, 0, 2.0);

	
	PM::Matrix sumVec_ref = PM::Matrix::Zero(3,refPtsCount);
	//PM::Matrix sumVec_ref(3,refPtsCount);
	//sumVec_ref.row(0) = PM::Matrix::Ones(1,refPtsCount);
	//sumVec_ref.bottomRows(2) = PM::Matrix::Zero(2,refPtsCount);

	PM::Matrix closeId_read = PM::Matrix::Constant(1,readPtsCount, -1);
	PM::Matrix weight_ref = PM::Matrix::Zero(1,refPtsCount);
	PM::Matrix sumDt_ref = PM::Matrix::Zero(1,refPtsCount);

	for(int i=0; i<readPtsCount; i++)
	{
		const Eigen::Vector3f pt = reading->features.col(i).head(3);
		const double timeRead = v_Msec_read(0,i)*1e6 + v_sec_read(0,i) + v_nsec_read(0,i)*1e-9;

		// Counters for reading
		float sumWeight = 0;
		float sumDt = 0;
		Eigen::Vector3f sumVec = Eigen::Vector3f::Zero(3,1);

		for(int k=0; k<knn; k++)
		{
			if(dists(k,i) != numeric_limits<float>::infinity())
			{
				const int id = ids(k,i);
				const Eigen::Vector3f pt_nn = reference->features.col(id).head(3);
				const double timeRef = v_Msec_ref(0,id)*1e6 + v_sec_ref(0,id) + v_nsec_ref(0,id)*1e-9;
				
				const Eigen::Vector3f delta = pt_nn - pt;
				const Eigen::Vector3f normal_read = v_normal_read.col(i);
				const double dt = abs(timeRead - timeRef);

				const float distRatio = sqrt(dists(k,i))/delta.norm();
				const float w_dist = 1 + eps - std::min(1.0f, distRatio);
				const float w_ang = angleWeight(normal_read, delta);
				//const float w = w_ang*w_dist;
				const float w = w_dist;
				//const float w = 1;

				assert(!isnan(w_dist));
				assert(!isnan(w_ang));
				assert(dt != 0);

			
				weight_ref(id)     += w;
				sumDt_ref(id)      += w*dt;
				sumVec_ref.col(id) += w*delta/dt;
				//sumVec_ref.col(id) += w*delta;

				sumWeight += w;
				sumDt     += w*dt;
				sumVec    += w*delta/dt;
				//sumVec    += w*delta;
				
				closeId_read(i) = id;
			}
		}

		// Weighted mean
		if(sumWeight > eps)
		{
			sumDt /= sumWeight;
			sumVec /= sumWeight;

			const float mag = sumVec.norm();
			if( mag < p_maxVelocity)
			{
				// Store in the descriptors
				v_velocityDir_read.col(i) = sumVec;
				v_velocityDt_read(0,i) = sumDt;
			}
		}



	}

	//------------------
	// Average points that received multiple matches on the reference
	for(int i=0; i<refPtsCount; i++)
	{
		// Avoid division per zero
		const float mag = sumVec_ref.col(i).norm();
		if(mag > eps)
		{
			// Weighted mean
			sumDt_ref(i) /= weight_ref(i);
			sumVec_ref.col(i) /= weight_ref(i);
			
			if(mag < p_maxVelocity)
			{
				// Store in the descriptors
				velocityDir_ref.col(i) = sumVec_ref.col(i);
				velocityDt_ref(0,i) = sumDt_ref(i);
			}	
		}
	}


	//------------------
	// Average all points with dual match
	for(int i=0; i<readPtsCount; i++)
	{
		assert(!isnan(v_velocityDir_read(0,i)));
		const int id = closeId_read(i);

		if(id != -1)
		{
			const float mag = velocityDir_ref.col(id).norm();
			if(mag > eps)
			{
				// We only keep the reference velocity, if only one match it will be the same
				v_velocityDir_read.col(i) = velocityDir_ref.col(id);
				v_velocityDt_read(0,i) = velocityDt_ref(0,id);
			}
		}
		
	}

	//------------------
	// Apply local constrains on velocity
	const int maxNN = 100;
	Matches::Dists dists_local(maxNN, readPtsCount);
	Matches::Ids ids_local(maxNN, readPtsCount);

	// Search
	read_tree->knn(static_read, ids_local, dists_local, maxNN, 0, 0, maxRadius);
	//PM::Matrix dists_nosqrt = dists_local.cwiseSqrt();
	//dists_local = dists_local.cwiseSqrt();
	
	// Initialize counters
	
	PM::Matrix sumVec_read = PM::Matrix::Zero(3,readPtsCount);

	PM::Matrix weight_read = PM::Matrix::Zero(1,readPtsCount);
	PM::Matrix sumMag_read = PM::Matrix::Zero(1,readPtsCount);
	PM::Matrix sumDt_read = PM::Matrix::Zero(1,readPtsCount);
	
	for(int i=0; i<readPtsCount; i++)
	{
		const Eigen::Vector3f normal_read = v_normal_read.col(i);
		
		for(int k=0; k<maxNN; k++)
		{
			if(dists_local(k,i) != numeric_limits<float>::infinity())
			{
				const int id = ids_local(k,i);
				const Eigen::Vector3f velocityDir = v_velocityDir_read.col(id);

				const float w_dist = eps + (1 - eps)*(1 - sqrt(dists_local(k,i))/maxRadius);
				assert(!isnan(w_dist));

				//const float w_ang = angleWeight(normal_read, velocityDir);
				//assert(!isnan(w_ang));

				const float w = w_dist;
				//const float w = 1;

				weight_read(i)     += w;
				sumVec_read.col(i) += w*velocityDir;
				sumDt_read(i)      += w*v_velocityDt_read(0,id);
			}
		}

	}
	
	//------------------
	// Average points given their neighborgs
	for(int i=0; i<readPtsCount; i++)
	{
		assert(!isnan(v_velocityDir_read(0,i)));
		// Avoid division per zero
		if(weight_read(i) > eps)
		{
			// Weighted mean
			sumDt_read(i) /= weight_read(i);
			sumVec_read.col(i) /= weight_read(i);

			const float mag = sumVec_read.col(i).norm();
			if(mag > eps && mag < p_maxVelocity)
			{
				// Store in the descriptors
				v_velocityDir_read.col(i) = sumVec_read.col(i);
				v_velocityDt_read(0,i) = sumDt_read(i);
			}
		}
		
		assert(isnan(v_velocityDir_read(0,i)) == false);

	}
	reading->addDescriptor("match_id", closeId_read);


	// Extra for debug
	DP projectedPointCloud = *reading;
	projectedPointCloud.features.topRows(3) = moved_read.topRows(3);
	debugPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(projectedPointCloud, p_mapFrame, ros::Time::now()));

	//usleep(100000);
}

float DynamicTrails::angleWeight(const Eigen::Vector3f &normal, const Eigen::Vector3f &velocity)
{
	const float projDist = min(1.0f, fabs(normal.dot(velocity.normalized())));
	assert(projDist <= 1);

	return (eps + (1 - eps)*(1 - 2.0*acos(projDist)/M_PI));

	//if(projDist > 0)
	//{
	//	return (1 + eps - 2.0*fabs(acos(min(1.0f, projDist)))/M_PI);
	//}
	//else
	//{
	//	const float projDistFlip = normal.dot(-velocity.normalized());
	//	return (1 + eps - 2.0*fabs(acos(min(1.0f, projDistFlip)))/M_PI);
	//}
}

void DynamicTrails::averageWithReference(shared_ptr<DP> reference, shared_ptr<DP> reading, const float dir)
{
	//const int readPtsCount = reading->features.cols();
	const int refPtsCount = reference->features.cols();
	
	//DP::View v_matchId_read = reading->getDescriptorViewByName("match_id");
	DP::View v_velocityDir_read = reading->getDescriptorViewByName("velocity_dir");
	DP::View v_velocityDt_read = reading->getDescriptorViewByName("velocity_dt");
	DP::View v_normal_read = reading->getDescriptorViewByName("normals");
	
	DP::View v_matchId_ref = reference->getDescriptorViewByName("match_id");
	DP::View v_velocityDir_ref = reference->getDescriptorViewByName("velocity_dir");
	DP::View v_velocityDt_ref = reference->getDescriptorViewByName("velocity_dt");
	
	for(int i=0; i<refPtsCount; i++)
	{
		const int id = v_matchId_ref(0,i);
		const float w_read = angleWeight(v_normal_read.col(id), v_velocityDir_read.col(id));
		const float w_ref = angleWeight(v_normal_read.col(id), v_velocityDir_ref.col(i));
		const float w_sum = w_read + w_ref;
		
		const float mag_ref = v_velocityDir_ref.col(i).norm();
		if(w_sum > eps && mag_ref > eps)
		{
			v_velocityDir_read.col(id) = (w_read*dir*v_velocityDir_read.col(id) - w_ref*dir*v_velocityDir_ref.col(i))/(w_sum);
			
			v_velocityDt_read(0,id) = (w_read*v_velocityDt_read(0,id) + w_ref*v_velocityDt_ref(0,i))/(w_sum);
		}
	}

	//for(int i=0; i<readPtsCount; i++)
	//{
	//	assert(!isnan(v_velocityDir_read(0,i)));
	//	const int id = v_matchId_read(0,i);
	//	const float w_read = angleWeight(v_normal_read.col(i), v_velocityDir_read.col(i));
	//	const float w_ref = angleWeight(v_normal_read.col(i), v_velocityDir_ref.col(id));
	//	const float w_sum = w_read + w_ref;
	//	
	//	const float mag_ref = v_velocityDir_ref.col(id).norm();
	//	if(w_sum > eps && mag_ref > eps)
	//	{
	//		assert((dir*v_velocityDir_read.col(i) - dir*v_velocityDir_ref.col(id)).norm() != 0);

	//		v_velocityDir_read.col(i) = (w_read*dir*v_velocityDir_read.col(i) - w_ref*dir*v_velocityDir_ref.col(id))/(w_sum);
	//		
	//		v_velocityDt_read(0,i) = (w_read*v_velocityDt_read(0,i) + w_ref*v_velocityDt_ref(0,id))/(w_sum);
	//	}
	//	assert(isnan(v_velocityDir_read(0,i)) == false);
	//}
}


// Main function supporting the DynamicTrails class
int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_maintainer");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	DynamicTrails trails(n, pn);

	ros::Rate r(50);
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
