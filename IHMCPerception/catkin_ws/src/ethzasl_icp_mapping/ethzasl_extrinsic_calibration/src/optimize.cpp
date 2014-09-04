// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#include <Eigen/Eigen>
#include <Eigen/LU>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <vector>
#include <map>

using namespace std;

static double inlierRatio(0.8);
static int restartCount(1);
static int generationCount(64);

double uniformRand()
{
	return double(rand())/RAND_MAX;
}

double gaussianRand(double mean, double sigm)
{
	// Generation using the Polar (Box-Mueller) method.
	// Code inspired by GSL, which is a really great math lib.
	// http://sources.redhat.com/gsl/
	// C++ wrapper available.
	// http://gslwrap.sourceforge.net/
	double r, x, y;

	// Generate random number in unity circle.
	do
	{
		x = uniformRand()*2 - 1;
		y = uniformRand()*2 - 1;
		r = x*x + y*y;
	}
	while (r > 1.0 || r == 0);

	// Box-Muller transform.
	return sigm * y * sqrt (-2.0 * log(r) / r) + mean;
}

struct TrainingEntry
{
	double timeStamp;
	Eigen::Vector3d odom_tr;
	Eigen::Quaterniond odom_rot;
	Eigen::Vector3d icp_tr;
	Eigen::Quaterniond icp_rot;

	TrainingEntry(){}
	TrainingEntry(std::istream& is)
	{
		is >> timeStamp;
		double t_x = 0, t_y = 0, t_z = 0, q_x = 0, q_y = 0, q_z = 0, q_w = 1;
		is >> t_x;
		is >> t_y;
		is >> t_z;
		icp_tr = Eigen::Vector3d(t_x, t_y, t_z);
		is >> q_x;
		is >> q_y;
		is >> q_z;
		is >> q_w;
		icp_rot = Eigen::Quaterniond(q_w, q_x, q_y, q_z).normalized();
		is >> t_x;
		is >> t_y;
		is >> t_z;
		odom_tr = Eigen::Vector3d(t_x, t_y, t_z);
		is >> q_x;
		is >> q_y;
		is >> q_z;
		is >> q_w;
		odom_rot = Eigen::Quaterniond(q_w, q_x, q_y, q_z).normalized();
		//odom_tr = odom_rot*odom_tr;
		//cerr << icp_rot.x() << " " << icp_rot.y() << " " << icp_rot.z() << " " << icp_rot.w() <<endl;
		//cerr << icp_tr.x() << " " << icp_tr.y() << " " << icp_tr.z() << endl;
		// FIXME: bug in Eigen ?
		//icp_tr = icp_rot*icp_tr;
		//cerr << icp_tr.x() << " " << icp_tr.y() << " " << icp_tr.z() << endl;
	}

	void dump(ostream& stream) const
	{
		stream << 
			timeStamp << " : " << 
			icp_tr.x() << " " << icp_tr.y() << " " << icp_tr.z() << " " <<
			icp_rot.x() << " " << icp_rot.y() << " " << icp_rot.z() << " " << icp_rot.w() << " " <<
			odom_tr.x() << " " << odom_tr.y() << " " << odom_tr.z() << " " <<
			odom_rot.x() << " " << odom_rot.y() << " " << odom_rot.z() << " " << odom_rot.w();
	}
};

struct TrainingSet: public std::vector<TrainingEntry>
{
	void dump()
	{
		for (TrainingSet::const_iterator it(begin()); it != end(); ++it)
		{
			const TrainingEntry& entry(*it);
			entry.dump(cout);
			cout << "\n";
		}
	}
};

TrainingSet trainingSet;

struct Params
{
	Eigen::Vector3d tr;
	Eigen::Quaterniond rot;
	
	Params():tr(0,0,0),rot(1,0,0,0) {}
	Params(const Eigen::Vector3d& _tr, const Eigen::Quaterniond& _rot):
		tr(_tr),
		rot(_rot)
		{}
	Params(const double transVariance):
		tr(
			gaussianRand(0, transVariance),
			gaussianRand(0, transVariance),
			gaussianRand(0, transVariance)
		),
		rot(
			Eigen::Quaterniond(1,0,0,0) *
			Eigen::Quaterniond(Eigen::AngleAxisd(uniformRand() * M_PI*2, Eigen::Vector3d::UnitX())) *
			Eigen::Quaterniond(Eigen::AngleAxisd(uniformRand() * M_PI*2, Eigen::Vector3d::UnitY())) *
			Eigen::Quaterniond(Eigen::AngleAxisd(uniformRand() * M_PI*2, Eigen::Vector3d::UnitZ()))
		)
		{}
	
	// add random noise
	const Params& mutate(const double amount = 1.0)
	{
		const double count = fabs(gaussianRand(0, 1));
		for (int i = 0; i < int(count + 1); i++)
		{
			const int toMutate = rand() % 6;
			switch (toMutate)
			{
				case 0: tr.x() += gaussianRand(0, 0.1) * amount; break;
				case 1: tr.y() += gaussianRand(0, 0.1) * amount; break;
				case 2: tr.z() += gaussianRand(0, 0.1) * amount; break;
				case 3: rot *= Eigen::Quaterniond(Eigen::AngleAxisd(gaussianRand(0, M_PI / 8) * amount, Eigen::Vector3d::UnitX())); break;
				case 4: rot *= Eigen::Quaterniond(Eigen::AngleAxisd(gaussianRand(0, M_PI / 8) * amount, Eigen::Vector3d::UnitY())); break;
				case 5: rot *= Eigen::Quaterniond(Eigen::AngleAxisd(gaussianRand(0, M_PI / 8) * amount, Eigen::Vector3d::UnitZ())); break;
				default: break;
			};
		}
		/*
		tr.x() += gaussianRand(0, 0.1) * amount;
		tr.y() += gaussianRand(0, 0.1) * amount;
		tr.z() += gaussianRand(0, 0.1) * amount;
		rot *= Eigen::Quaterniond(Eigen::AngleAxisd(gaussianRand(0, M_PI / 8) * amount, Eigen::Vector3d::UnitX()));
		rot *= Eigen::Quaterniond(Eigen::AngleAxisd(gaussianRand(0, M_PI / 8) * amount, Eigen::Vector3d::UnitY()));
		rot *= Eigen::Quaterniond(Eigen::AngleAxisd(gaussianRand(0, M_PI / 8) * amount, Eigen::Vector3d::UnitZ()));
		*/
		return *this;
	}
	
	// renormalize quaternion
	void normalize()
	{
		rot.normalize();
		if (rot.x() < 0)
		{
			rot.x() = -rot.x();
			rot.y() = -rot.y();
			rot.z() = -rot.z();
			rot.w() = -rot.w();
		}
	}
	
	// dump content of this transform
	void dumpTransform(ostream& stream) const
	{
		stream << tr.x() << " " << tr.y() << " " << tr.z() << " " <<
				rot.x() << " " << rot.y() << " " << rot.z() << " " << rot.w();
	}
	
	// dump what to copy/paste
	void dumpCopyPaste(ostream& stream) const
	{
		/*stream <<
			"translation: x=" << tr.x() << " y=" << tr.y() << " z=" << tr.z() << " " <<
			"quaternion: x=" << rot.x() << " y=" << rot.y() << " z=" << rot.z() << " w=" << rot.w();
		*/
		stream << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"base_link_to_kinect\" args=\"";
		dumpTransform(stream);
		stream << " /base_link /kinect 100\"/>" 
		" \n\n OR \n\n"
		"rosrun tf static_transform_publisher ";
		dumpTransform(stream);
		stream << " /base_link /kinect 100\n";
	}
};

// we have two typedefs for the same physical object because they are conceptually different
typedef vector<Params> Genome;
typedef vector<Params> ParamsVector;

void normalizeParams(ParamsVector& params)
{
	for (size_t i = 0; i < params.size(); ++i)
		params[i].normalize();
}

void dumpParamsStats(ostream& stream, const ParamsVector& params)
{
	Eigen::Vector3d trMean(0,0,0);
	Eigen::Quaterniond::Coefficients rotMean(0,0,0,0);
	for (size_t i = 0; i < params.size(); ++i)
	{
		trMean += params[i].tr;
		rotMean += params[i].rot.coeffs();
	}
	trMean /= double(params.size());
	rotMean /= double(params.size());
	
	Params mean(trMean,Eigen::Quaterniond(rotMean));
	mean.normalize();
	mean.dumpCopyPaste(stream); stream << "\n";
	
	Eigen::Vector3d trVar(0,0,0);
	Eigen::Quaterniond::Coefficients rotVar(0,0,0,0);
	for (size_t i = 0; i < params.size(); ++i)
	{
		trVar += (params[i].tr - trMean).cwiseProduct(params[i].tr - trMean);
		rotVar += (params[i].rot.coeffs() - rotMean).cwiseProduct(params[i].rot.coeffs() - rotMean);
	}
	trVar /= double(params.size());
	rotVar /= double(params.size());
	
	stream << "\nVariance:\n";
	Params var(trVar,Eigen::Quaterniond(rotVar));
	var.dumpTransform(stream); stream << "\n";
	
	stream << "\nValues:\n";
	for (size_t i = 0; i < params.size(); ++i)
	{
		params[i].dumpTransform(stream); stream << "\n";
	}
	stream << endl;
}

double computeError(const Params& p, const TrainingEntry& e)
{
	/*
	// version with Eigen::Matrix4d
	Eigen::Matrix4d blk(Eigen::Matrix4d::Identity());
	blk.corner(Eigen::TopLeft,3,3) = p.rot.toRotationMatrix();
	blk.corner(Eigen::TopRight,3,1) = p.tr;
	
	Eigen::Matrix4d odom(Eigen::Matrix4d::Identity());
	odom.corner(Eigen::TopLeft,3,3) = e.odom_rot.toRotationMatrix();
	odom.corner(Eigen::TopRight,3,1) = e.odom_tr;
	
	Eigen::Matrix4d blk_i(blk.inverse());
	
	//const Eigen::Matrix4d pred_icp = blk * odom * blk_i;
	const Eigen::Matrix4d pred_icp = blk_i * odom * blk;
	
	const Eigen::Matrix3d pred_icp_rot_m = pred_icp.corner(Eigen::TopLeft,3,3);
	const Eigen::Quaterniond pred_icp_rot = Eigen::Quaterniond(pred_icp_rot_m);
	const Eigen::Vector3d pred_icp_tr = pred_icp.corner(Eigen::TopRight,3,1);
	*/
	
	// version with Eigen::Transform3d
	typedef Eigen::Transform<double, 3, Eigen::Affine> Transform3d;
	typedef Eigen::Translation<double, 3> Translation3d;
	
	const Transform3d blk = Translation3d(p.tr) * p.rot;
	const Transform3d blk_i = Transform3d(blk.inverse(Eigen::Isometry));
	const Transform3d odom = Translation3d(e.odom_tr) * e.odom_rot;
	//const Eigen::Transform3d pred_icp = blk * odom * blk_i;
	const Transform3d pred_icp = blk_i * odom * blk;
	
	const Eigen::Matrix3d pred_icp_rot_m = pred_icp.matrix().topLeftCorner(3,3);
	const Eigen::Quaterniond pred_icp_rot = Eigen::Quaterniond(pred_icp_rot_m);
	const Eigen::Vector3d pred_icp_tr = pred_icp.translation();
	
	
	// identity checked ok
	//cerr << "mat:\n" << blk.matrix() * blk_i.matrix() << endl;
	
	//cout << "dist: " << (e.icp_tr - pred_icp.translation()).norm() << endl;
	//cout << "ang dist: " << e.icp_rot.angularDistance(pred_icp_rot) << endl;
	//cout << "tr pred icp:\n" << pred_icp.translation() << "\nicp:\n" << e.icp_tr << "\n" << endl;
	//cout << "rot pred icp:\n" << pred_icp_rot << "\nicp:\n" << e.icp_rot << "\n" << endl;
	// FIXME: tune coefficient for rot vs trans
	
	const double e_tr((e.icp_tr - pred_icp_tr).norm());
	const double e_rot(e.icp_rot.angularDistance(pred_icp_rot));
	/*if (e_tr < 0)
		abort();
	if (e_rot < 0)
		abort();*/
	//cerr << e_tr << " " << e_rot << endl;
	return e_tr + e_rot;
}

// compute errors with outlier rejection
double computeError(const Params& p)
{
	vector<double> errors;
	errors.reserve(trainingSet.size());
	for (TrainingSet::const_iterator it(trainingSet.begin()); it != trainingSet.end(); ++it)
	{
		const TrainingEntry& entry(*it);
		errors.push_back(computeError(p, entry));
	}
	sort(errors.begin(), errors.end());
	const size_t inlierCount(errors.size() * inlierRatio);
	double error = 0;
	for (size_t i = 0; i < inlierCount; ++i)
		error += errors[i];
	return error;
}

double evolveOneGen(Genome& genome, double annealing = 1.0, Params* bestParams = 0)
{
	typedef multimap<double, Params> EvaluationMap;
	typedef EvaluationMap::iterator EvaluationMapIterator;
	EvaluationMap evalutationMap;

	double totalError = 0;
	double bestError = numeric_limits<double>::max();
	int bestInd = 0;
	for (size_t ind = 0; ind < genome.size(); ind++)
	{
		const double error = computeError(genome[ind]);
		if (error < bestError)
		{
			bestError = error;
			bestInd = ind;
		}

		totalError += error;
		evalutationMap.insert(make_pair(error, genome[ind]));

		/*cout << "E " << ind << " : ";
		genome[ind].dump(cout);
		cout << " = " << error << "\n";*/
	}

	if (bestParams)
		*bestParams = genome[bestInd];

	assert((genome.size() / 4) * 4 == genome.size());

	size_t ind = 0;
	for (EvaluationMapIterator it = evalutationMap.begin(); ind < genome.size() / 4; ++it, ++ind )
	{
		//cout << "S " << it->first << "\n";
		genome[ind * 4] = it->second;
		genome[ind * 4 + 1] = it->second.mutate(annealing);
		genome[ind * 4 + 2] = it->second.mutate(annealing);
		genome[ind * 4 + 3] = it->second.mutate(annealing);
	}

	return bestError;
}

int main(int argc, char** argv)
{
	srand(time(0));
	
	if (argc < 2)
	{
		cerr << "Usage " << argv[0] << " LOG_FILE_NAME [INLIER_RATIO] [RESTART_COUNT] [GEN_COUNT]\n";
		cerr << "  LOG_FILE_NAME   name of file to load containing delta tf\n";
		cerr << "  INLIER_RATIO    ratio of inlier to use for error computation (range: ]0:1], default: 0.8)\n";
		cerr << "  RESTART_COUNT   number of random restart (default: 1)\n";
		cerr << "  GEN_COUNT       number of generation for the ES (default: 64)\n";
		cerr << endl;
		return 1;
	}
	if (argc >= 3)
	{
		inlierRatio = atof(argv[2]);
		if (inlierRatio <= 0 || inlierRatio > 1)
		{
			cerr << "Invalid inline ratio: " << inlierRatio << ", must be within range ]0:1]" << endl;
			return 2;
		}
	}
	cout << "Inlier ratio: " << inlierRatio << endl;
	if (argc >= 4)
	{
		restartCount = atoi(argv[3]);
		if (restartCount < 1)
		{
			cerr << "Invalid restart count: " << restartCount << ", must be greater than 0" << endl;
			return 3;
		}
	}
	cout << "Restart count: " << restartCount << endl;
	if (argc >= 5)
	{
		generationCount = atoi(argv[4]);
		if (generationCount < 1)
		{
			cerr << "Invalid generation count: " << generationCount << ", must be greater than 0" << endl;
			return 4;
		}
	}
	cout << "Generation count: " << generationCount << endl;
	
	ifstream ifs(argv[1]);
	while (ifs.good())
	{
		TrainingEntry e(ifs);
		if (ifs.good())
		{
			trainingSet.push_back(e);
			//e.dump(cerr); cerr << endl;
		}
	}
	cout << "Loaded " << trainingSet.size() << " training entries" << endl;
	//trainingSet.dump();
	
	ParamsVector bests;
	for (int i = 0; i < restartCount; ++i)
	{
		cout << "Starting " << i << " restart on " << restartCount << endl;
		Genome genome(1024);
		for (size_t g = 0; g < genome.size(); ++g)
			genome[g] = Params(0.5);
		for (int g = 0; g < generationCount; ++g)
		{
			cout << "\r" << g << " best has error " << evolveOneGen(genome, 2. * (double)(generationCount - g) / (double)(generationCount)) << "            ";
			cout.flush();
		}
		Params best;
		cout << "\r  Best error of restart " << i << ": " << evolveOneGen(genome, 1.0, &best) << endl;
		bests.push_back(best);
	}
	
	cout << "\nOptimization completed, code to COPY-PASTE in to use the transformation:\n\n";
	if (restartCount == 1)
	{
		bests[0].dumpCopyPaste(cout);
	}
	else
	{
		normalizeParams(bests);
		dumpParamsStats(cout, bests);
	}
	
	return 0;
}
