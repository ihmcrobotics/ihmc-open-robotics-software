#include "FactorGraphExternal.h"

void FactorGraphExternal::addPriorPoseFactor(int index, float *pose)
{
    using namespace gtsam;
    Pose3 initPose(Rot3::Ypr(pose[0], pose[1], pose[2]), Point3(pose[3], pose[4], pose[5]));
    factorGraphHandler.addPriorPoseFactor(index, initPose);
}

void FactorGraphExternal::addOdometryFactor(float *odometry, int poseId)
{
    using namespace gtsam;
    Pose3 odometryValue(Rot3::Ypr(odometry[0], odometry[1], odometry[2]), Point3(odometry[3], odometry[4], odometry[5]));
    factorGraphHandler.addOdometryFactor(odometryValue, poseId);
}

void FactorGraphExternal::addOdometryFactorExtended(double *odometry, int poseId)
{
    using namespace gtsam;
    Eigen::Matrix4d M = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >(odometry);

    std::cout << "Add Odometry Factor Extended: " << std::endl << M << std::endl;

    Pose3 odometryValue(M);
    factorGraphHandler.addOdometryFactor(odometryValue, poseId);
}

void FactorGraphExternal::addOrientedPlaneFactor(float *lmMean, int lmId, int poseIndex)
{
}

void FactorGraphExternal::addGenericProjectionFactor(float *point, int lmId, int poseIndex)
{
    factorGraphHandler.addGenericProjectionFactor(gtsam::Point2(point[0], point[1]), lmId, poseIndex);
}

void FactorGraphExternal::setPointLandmarkInitialValue(int landmarkId, float* value)
{
    factorGraphHandler.setPointLandmarkInitialValue(landmarkId, {value[0], value[1], value[2]});
}

void FactorGraphExternal::optimize()
{
    factorGraphHandler.optimize();
}

void FactorGraphExternal::optimizeISAM2(uint8_t numberOfUpdates)
{
    factorGraphHandler.optimizeISAM2(numberOfUpdates);
}

void FactorGraphExternal::clearISAM2()
{
    factorGraphHandler.clearISAM2();
}

void FactorGraphExternal::setPoseInitialValue(int index, float *value)
{
    using namespace gtsam;
    Pose3 initialValue(Rot3::Ypr(value[0], value[1], value[2]), Point3(value[3], value[4], value[5]));
    factorGraphHandler.setPoseInitialValue(index, initialValue);
}

void FactorGraphExternal::setPoseInitialValueExtended(int index, float *value)
{
    using namespace gtsam;
    Eigen::Matrix4f M = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor> >(value);

    std::cout << "Set Pose Initial Extended: " << std::endl << M << std::endl;

    Pose3 initialValue(M.cast<double>());
    factorGraphHandler.setPoseInitialValue(index, initialValue);
}

void FactorGraphExternal::setOrientedPlaneInitialValue(int landmarkId, float *value)
{
}

void FactorGraphExternal::createOdometryNoiseModel(float *odomVariance)
{
}

void FactorGraphExternal::createOrientedPlaneNoiseModel(float *lmVariances)
{
}

void FactorGraphExternal::getResultPoses(double* poses, uint32_t* poseIDs, uint32_t count)
{
    for(uint32_t i = 0; i<count; i++)
    {
        auto matrix = factorGraphHandler.getResults().at<gtsam::Pose3>(gtsam::Symbol('x', poseIDs[i])).matrix();

        matrix.transposeInPlace();

        std::cout << "GetResultPoses" << std::endl << matrix << std::endl;
        std::copy(  matrix.data(),
                    matrix.data() + 16,
                    poses + 16 * i);
    }
}

void FactorGraphExternal::getResultLandmarks(double* landmarks, uint32_t* landmarkIDs, uint32_t count)
{
    for(uint32_t i = 0; i<count; i++)
    {
        std::copy(  factorGraphHandler.getResults().at<gtsam::Point3>(gtsam::Symbol('p', landmarkIDs[i])).data(),
                    factorGraphHandler.getResults().at<gtsam::Point3>(gtsam::Symbol('p', landmarkIDs[i])).data() + 3,
                    landmarks + 3 * i);
    }
}

void FactorGraphExternal::printResults()
{
    factorGraphHandler.getResults().print();
}

void FactorGraphExternal::helloWorldTest()
{
    std::cout << "Hello from native code" << std::endl;

    for (int i = 0; i < 5; i++)
    {
       std::cout << "Hello " << i << std::endl;
    }
}

void FactorGraphExternal::visualSLAMTest()
{
    factorGraphHandler.VisualSLAMTest();
}
