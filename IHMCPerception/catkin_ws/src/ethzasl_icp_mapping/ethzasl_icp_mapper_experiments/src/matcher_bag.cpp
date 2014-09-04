
#define EIGEN2_SUPPORT_STAGE30_FULL_EIGEN3_API
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <map>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include "boost/foreach.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/tfMessage.h"
#include "tf2/buffer_core.h"
#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include "sensor_msgs/CameraInfo.h"
#include <argtable2.h>
#include <regex.h>      /* REG_ICASE */
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"
#include "pointmatcher_ros/point_cloud.h"

using namespace std;
using namespace Eigen;
using namespace PointMatcherSupport;

typedef map<string, string> Params;

// FIXME: use yaml ?

Params params;

bool hasParam(const std::string& name)
{
	return params.find(name) != params.end();
}

template<typename T>
T getParam(const std::string& name, const T& defaultValue)
{
	Params::const_iterator it(params.find(name));
	if (it != params.end())
	{
		T v;
		istringstream iss(it->second);
		iss >> v;
		cerr << "Found parameter: " << name << ", value: " << v << endl;
		return v;
	}
	else
	{
		cerr << "Cannot find value for parameter: " << name << ", assigning default: " << defaultValue << endl;
		return defaultValue;
	}
}

template<typename T>
T getParam(const std::string& name)
{
	Params::const_iterator it(params.find(name));
	if (it != params.end())
	{
		T v;
		istringstream iss(it->second);
		iss >> v;
		cerr << "Found parameter: " << name << ", value: " << v << endl;
		return v;
	}
	else
	{
		cerr << "Cannot find value for parameter: " << name << endl;
		return T();
	}
}

void reloadParamsFromLine(ifstream& ifs)
{
	char line[65536];
	ifs.getline(line, sizeof(line));
	params.clear();
	istringstream iss(line);
	while (iss.good())
	{
		string keyVal;
		iss >> keyVal;
		if (!keyVal.empty())
		{
			const size_t delPos = keyVal.find_first_of('=');
			if (delPos != string::npos)
			{
				const string key = keyVal.substr(0, delPos);
				const string val = keyVal.substr(delPos+1);
				params[key] = val;
			}
		}
	}
}

void openWriteFile(ofstream& stream, arg_file * file, const int errorVal)
{
	if (file->count > 0)
	{
		stream.open(file->filename[0]);
		if (!stream.good())
		{
			cerr << "Error, invalid " << file->hdr.glossary << " " << file->filename[0] << endl;
			exit (errorVal);
		}
		else
			cout << "Writing to " << file->hdr.glossary << " " << file->filename[0] << endl;
	}
}

#include "icp_chain_creation.h"

struct Datum
{
	TP transform;
	DP cloud;
	ros::Time stamp;
};

typedef vector<Datum> Data;

TP T_v_to_k(TP::Identity(4, 4));
TP T_k_to_v(TP::Identity(4, 4));

TP msgTransformToTP(const geometry_msgs::Transform& transform)
{
	const geometry_msgs::Vector3 gv(transform.translation);
	const geometry_msgs::Quaternion gq(transform.rotation);
	const Vector3 vect(gv.x, gv.y, gv.z);
	const Quaternion<Scalar> quat(gq.w, gq.x, gq.y, gq.z);
	const Transform<Scalar,3,Affine> tr = Translation<Scalar,3>(vect) * quat;
	return tr.matrix();
}

int main(int argc, char **argv)
{
	arg_file * bagFile = arg_file1(NULL, NULL, "<bag>", "bag file (mandatory)");
	arg_str  * cloudTopic = arg_str1(NULL, NULL, "<cloud_topic>", "cloud topic (mandatory)");
	arg_file * paramsFile = arg_file1(NULL, NULL, "<params>", "params file (mandatory)");
	arg_file * statFile = arg_file0(NULL, "stat", NULL, "statistic output file (default: no output)");
	arg_int  * onlyFirsts = arg_int0(NULL, "only_firsts", "<N>", "use only first <N> clouds (default: unlimited)");
	
	arg_rex  * cmdGt = arg_rex1(NULL, NULL, "gt", NULL, REG_ICASE, "use ground truth");

	arg_str  * gtTargetFrame = arg_str1(NULL, "gt_target_frame", NULL, "tf target frame of ground truth, without heading \"/\" (mandatory)");
	arg_str  * gtSourceFrame = arg_str1(NULL, "gt_source_frame", NULL, "tf source frame of ground truth, without heading \"/\" (mandatory)");
	
	arg_str  * corr = arg_str0(NULL, "corr", NULL, "correction from gt to icp, as a string containing \"t_x t_y t_z q_x q_y q_z q_w\" (default: no correction)");
	
	arg_file * tfFile = arg_file0(NULL, "tf", NULL, "tf output file (default: no output)");
	arg_file * dtfFile = arg_file0(NULL, "dtf", NULL, "delta tf output file (default: no output)");
	arg_int  * dtfSteps = arg_int0(NULL, "dtf_steps", NULL, "steps for computing delta tf (default: 30)");
	
	struct arg_end  * end = arg_end(20);
	
	void *argTableCloud[] = {bagFile, cloudTopic, paramsFile, onlyFirsts, statFile, tfFile, end};
	void* argTableCloudGt[] = {bagFile, cloudTopic, paramsFile, cmdGt, gtTargetFrame, gtSourceFrame, corr, onlyFirsts, statFile, tfFile, dtfFile, dtfSteps, end};
		
	if (arg_nullcheck(argTableCloud) != 0 || 
		arg_nullcheck(argTableCloudGt) != 0)
		abort();
	
	const int nerrorsCloudGt = arg_parse(argc,argv,argTableCloudGt);
	const int nerrorsCloud = arg_parse(argc,argv,argTableCloud);
	if (nerrorsCloud > 0 && nerrorsCloudGt > 0)
	{
		cout << "Error(s) in command line:\n";
		arg_print_errors(stdout,end,argv[0]);
		cout << "\n";
		cout << "Usage, for cloud without ground-truth:\n";
		cout << argv[0];
		arg_print_syntax(stdout,argTableCloud,"\n");
		arg_print_glossary(stdout,argTableCloud,"  %-28s %s\n");
		cout << "\n";
		cout << "Usage, for cloud with ground-truth:\n";
		cout << argv[0];
		arg_print_syntax(stdout,argTableCloudGt,"\n");
		arg_print_glossary(stdout,argTableCloudGt,"  %-28s %s\n");
		return 1;
	}
	
	const bool useGt = (nerrorsCloudGt == 0);
	
	srand(time(0));
	
	// setup correction
	if (useGt)
	{
		if (corr->count > 0)
		{
			istringstream iss(corr->sval[0]);
			Eigen::Vector3f tr;
			iss >> tr(0);
			iss >> tr(1);
			iss >> tr(2);
			Eigen::eigen2_Quaternionf rot;
			iss >> rot.x();
			iss >> rot.y();
			iss >> rot.z();
			iss >> rot.w();
			T_k_to_v = (Eigen::eigen2_Translation3f(tr) * rot).matrix();
			T_v_to_k = T_k_to_v.inverse();
			cout << "Using ground-truth with correction:\n";
			cout << T_k_to_v << "\n";
		}
		else
			cout << "Using ground-truth without correction" << endl;
	}
	else
		cout << "Not using ground-truth" << endl;
	
	// load param list
	ifstream ifs(paramsFile->filename[0]);
	if (!ifs.good())
	{
		cerr << "Error, invalid params file " << paramsFile->filename[0] << endl;
		return 3;
	}
	
	// open stat and tf output files, if requested
	ofstream statofs, tfofs, dtfofs;
	openWriteFile(statofs, statFile, 4);
	openWriteFile(tfofs, tfFile, 5);
	openWriteFile(dtfofs, dtfFile, 6);
	
	int deltaTfSteps(30);
	if (useGt && dtfSteps->count > 0)
	{
		deltaTfSteps = dtfSteps->ival[0];
		cout << "Using tf steps of " << deltaTfSteps << endl;
	}
	
	// load data
	try
	{
		sensor_msgs::CameraInfo camInfo;
		
		// 1 year of buffer
		tf2::BufferCore tfBuffer(ros::Duration(31536000,0));
		if (useGt)
		{
			// first pass, read tf
			rosbag::Bag bag(bagFile->filename[0]);
			
			unsigned tfCount(0);
			rosbag::View view(bag, rosbag::TopicQuery("/tf"));
			BOOST_FOREACH(rosbag::MessageInstance const m, view)
			{
				typedef geometry_msgs::TransformStamped TS;
				tf::tfMessage::ConstPtr tfMsg = m.instantiate<tf::tfMessage>();
				for (tf::tfMessage::_transforms_type::const_iterator it = tfMsg->transforms.begin(); it != tfMsg->transforms.end(); ++it)
				{
					const TS& ts(*it);
					if (tfCount % 100 == 0)
					{
						cout << "\rLoading tf: " << tfCount << " loaded...";
						cout.flush();
					}
					//cout << ts.header.stamp << ":" << ts.header.frame_id << " <- " << ts.child_frame_id << endl;
					tfBuffer.setTransform(ts, "default");
					++tfCount;
				}
			}
			bag.close();
			cout << "\r" << tfCount << " tf loaded             " << endl;
		}
		
		// create labels
		DP::Labels labels;
		labels.push_back(DP::Label("x", 1));
		labels.push_back(DP::Label("y", 1));
		labels.push_back(DP::Label("z", 1));
		labels.push_back(DP::Label("pad", 1));
		
		// for each line in the experiment file
		unsigned expCount(0);
		while (ifs.good())
		{
			// read line and load params
			reloadParamsFromLine(ifs);
			if (params.empty())
				break;
			cout << "Exp " << expCount << ", loaded " << params.size() << " parameters:\n";
			++expCount;
			
			// run experiment
			PM::ICPSequence icp;
			populateParameters(icp);
			cout << endl;
			unsigned failCount(0);
			unsigned processedCount(0);
			double icpTotalDuration(0);
			TP T_gt_init;
			TP T_gt_old;
			TP T_icp_old;
			
			TP T_d_gt_acc(TP::Identity(4,4));
			TP T_d_icp_acc(TP::Identity(4,4));
			
			// open bag
			rosbag::Bag bag(bagFile->filename[0]);
			unsigned cloudLimit(onlyFirsts->count > 0 ? onlyFirsts->ival[0] : numeric_limits<unsigned>::max());
			unsigned cloudCount(0);
			rosbag::View view(bag, rosbag::TopicQuery(cloudTopic->sval[0]));
			BOOST_FOREACH(rosbag::MessageInstance const m, view)
			{
				// make sure we have not reached the limit
				if (cloudCount >= cloudLimit)
					break;
				
				// get cloud message
				sensor_msgs::PointCloud2::ConstPtr cloudMsg = m.instantiate<sensor_msgs::PointCloud2>();
				if (cloudMsg == 0)
				{
					cerr << "E " << cloudCount << " : Null cloud message after deserialization, are you sure the cloud topic is really " << cloudTopic->sval[0] << " ?" << endl;
					abort();
				}
				
				// getting transform, if gt is used
				string err;
				TP T_gt = TP::Identity(4,4);
				if (useGt)
				{
					if (tfBuffer.canTransform(gtTargetFrame->sval[0], gtSourceFrame->sval[0], cloudMsg->header.stamp, &err))
					{
						const geometry_msgs::TransformStamped ts = tfBuffer.lookupTransform(gtTargetFrame->sval[0], gtSourceFrame->sval[0], cloudMsg->header.stamp);
						T_gt = msgTransformToTP(ts.transform);
						cout << "* " << cloudCount << " : Got transform from " << gtSourceFrame->sval[0] << " to " << gtTargetFrame->sval[0] << " at time: " <<  cloudMsg->header.stamp << endl;
					}
					else
					{
						cout << "W " << cloudCount << " : Cannot get transform from " << gtSourceFrame->sval[0] << " to " << gtTargetFrame->sval[0] << " at time: " <<  cloudMsg->header.stamp << endl;
						++cloudCount;
						continue;
					}
				}
				
				// create cloud
				++processedCount;
				DP d(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*cloudMsg));
				cout << "  Using " << d.features.cols() << " good points on " << cloudMsg->width * cloudMsg->height << "\n";;
				
				// apply icp
				bool mustInitICP(!icp.hasKeyFrame());
				timer t;
				try
				{
					icp(d);
					icpTotalDuration += t.elapsed();
				}
				catch (PM::ConvergenceError error)
				{
					icpTotalDuration += t.elapsed();
					++failCount;
					cerr << "W ICP failed to converge at cloud " << cloudCount << " : " << error.what() << endl;
				}
				
				// if icp has no key frame, init transforms
				if (mustInitICP)
				{
					T_gt_init = T_gt;
					T_gt_old = T_gt;
				}
				
				// compute T given ICP in ground-truth coordinates
				const TP T_icp = T_gt_init * T_k_to_v * icp.getTransform() * T_v_to_k;
				
				// if icp has no key frame, init transforms
				if (mustInitICP)
					T_icp_old = T_icp;
				
				// write tf output
				if (tfofs.good())
				{
					tfofs << cloudMsg->header.stamp << " ";
					
					const Vector3 t_icp(T_icp.topRightCorner(3,1));
					const Quaternion<Scalar> q_icp(Matrix3(T_icp.topLeftCorner(3,3)));
					tfofs << t_icp(0) << " " << t_icp(1) << " " << t_icp(2) << " " << q_icp.x() << " " << q_icp.y() << " " << q_icp.z() << " " << q_icp.w();
					if (useGt)
					{
						const Vector3 t_gt(T_gt.topRightCorner(3,1));
						const Quaternion<Scalar> q_gt(Matrix3(T_gt.topLeftCorner(3,3)));
						tfofs << " " << t_gt(0) << " " << t_gt(1) << " " << t_gt(2) << " " << q_gt.x() << " " << q_gt.y() << " " << q_gt.z() << " " << q_gt.w();
					}
					tfofs << "\n";
				}
				
				// if not the first frame
				if (!mustInitICP)
				{
					// compute ground-truth transfrom
					const TP T_d_gt = T_gt.inverse() * T_gt_old;
					// compute icp inverse transform
					const TP T_d_icp = T_icp.inverse() * T_icp_old;
					
					// compute errors
					const TP T_d = T_d_gt * T_d_icp.inverse();
					if (useGt)
					{
						const Vector3 e_t(T_d.topRightCorner(3,1));
						icp.inspector->addStat("e_x", e_t(0));
						icp.inspector->addStat("e_y", e_t(1));
						icp.inspector->addStat("e_z", e_t(2));
						const Quaternion<Scalar> quat(Matrix3(T_d.topLeftCorner(3,3)));
						icp.inspector->addStat("e_a", 2 * acos(quat.normalized().w()));
					}
					
					if (cloudCount % deltaTfSteps == 0)
					{
						if (useGt)
						{
							// compute difference
							const TP T_d_acc = T_d_gt_acc * T_d_icp_acc.inverse();
							
							// compute errors
							const Vector3 e_t(T_d_acc.topRightCorner(3,1));
							icp.inspector->addStat("e_acc_x", e_t(0));
							icp.inspector->addStat("e_acc_y", e_t(1));
							icp.inspector->addStat("e_acc_z", e_t(2));
							const Quaternion<Scalar> quat(Matrix3(T_d_acc.topLeftCorner(3,3)));
							icp.inspector->addStat("e_acc_a", 2 * acos(quat.normalized().w()));
						}
						
						// dump deltas
						if (dtfofs.good())
						{
							dtfofs << cloudMsg->header.stamp << " ";
							
							const Vector3 t_icp(T_d_icp_acc.topRightCorner(3,1));
							const Quaternion<Scalar> q_icp(Matrix3(T_d_icp_acc.topLeftCorner(3,3)));
							dtfofs << t_icp(0) << " " << t_icp(1) << " " << t_icp(2) << " " << q_icp.x() << " " << q_icp.y() << " " << q_icp.z() << " " << q_icp.w();
							if (useGt)
							{
								const Vector3 t_gt(T_d_gt_acc.topRightCorner(3,1));
								const Quaternion<Scalar> q_gt(Matrix3(T_d_gt_acc.topLeftCorner(3,3)));
								dtfofs << " " << t_gt(0) << " " << t_gt(1) << " " << t_gt(2) << " " << q_gt.x() << " " << q_gt.y() << " " << q_gt.z() << " " << q_gt.w();
							}
							dtfofs << "\n";
						}
						
						// reset accumulators
						T_d_gt_acc = TP::Identity(4,4);
						T_d_icp_acc = TP::Identity(4,4);
					}
					else
					{
						T_d_gt_acc = T_d_gt * T_d_gt_acc;
						T_d_icp_acc = T_d_icp * T_d_icp_acc;
					}
				}
				
				T_gt_old = T_gt;
				T_icp_old = T_icp;
				
				++cloudCount;
			}
			bag.close();
			
			if (cloudCount == 0)
			{
				cerr << "E : No cloud processed, are you sure the cloud topic is really " << cloudTopic->sval[0] << " ?" << endl;
				return 0;
			}
			
			if (failCount > 0)
			{
				cerr << "W : " << failCount << " registrations failed on " << processedCount-1 << " (" << 100.*double(failCount)/double(processedCount-1) << " %)" << endl;
			}
			
			// write general stats
			if (statofs.good())
			{
				statofs << processedCount-1 << " " << failCount << " * ";
				statofs << icpTotalDuration << " * ";
				icp.inspector->dumpStats(statofs);
				statofs << " # ";
				// params for info
				for (Params::const_iterator it(params.begin()); it != params.end(); ++it)
					statofs << it->first << "=" << it->second << " ";
				// finish results
				statofs << "\n";
			}
		}
	}
	catch (rosbag::BagException e)
	{
		cerr << "Error, error reading bag file: " << e.what() << endl;
		return 2;
	}
	
	return 0;
}
