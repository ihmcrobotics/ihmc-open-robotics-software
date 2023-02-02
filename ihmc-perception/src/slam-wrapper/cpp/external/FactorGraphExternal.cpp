#include "FactorGraphExternal.h"

void FactorGraphExternal::addPriorPoseFactor(int index, float *pose)
{
    using namespace gtsam;
    Pose3 initPose(Rot3::Ypr(pose[0], pose[1], pose[2]), Point3(pose[3], pose[4], pose[5]));
    factorGraphHandler.addPriorPoseFactor(index, initPose);
}

void FactorGraphExternal::addPriorPoseFactorExtended(int poseId, double *pose)
{
    using namespace gtsam;
    Eigen::Matrix4d M = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >(pose);

    Pose3 poseValue(M);
    factorGraphHandler.addPriorPoseFactor(poseId, poseValue);
}

void FactorGraphExternal::addOdometryFactor(int poseId, float *odometry)
{
    using namespace gtsam;
    Pose3 odometryValue(Rot3::Ypr(odometry[0], odometry[1], odometry[2]), Point3(odometry[3], odometry[4], odometry[5]));
    factorGraphHandler.addOdometryFactor(odometryValue, poseId);
}

void FactorGraphExternal::addOdometryFactorExtended(int poseId, double *odometry)
{
    using namespace gtsam;
    Eigen::Matrix4d M = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >(odometry);

    Pose3 odometryValue(M);
    factorGraphHandler.addOdometryFactor(odometryValue, poseId);
}

void FactorGraphExternal::addOrientedPlaneFactor(int lmId, int poseIndex, float *lmMean)
{
    using namespace gtsam;
    Vector4 planeValue(lmMean[0], lmMean[1], lmMean[2], lmMean[3]);
    factorGraphHandler.addOrientedPlaneFactor(planeValue, lmId, poseIndex);
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

void FactorGraphExternal::setPoseInitialValueExtended(int index, double *value)
{
    printf("setPoseInitialValueExtended(%d)\n", index); fflush(stdout);
    using namespace gtsam;
    Eigen::Matrix4d M = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >(value);

    // std::cout << "Set Pose Initial Extended: " << std::endl << M << std::endl;

    Pose3 initialValue(M);
    factorGraphHandler.setPoseInitialValue(index, initialValue);
}

void FactorGraphExternal::setOrientedPlaneInitialValue(int landmarkId, float *value)
{
    using namespace gtsam;
    OrientedPlane3 initialValue(value[0], value[1], value[2], value[3]);
    factorGraphHandler.setOrientedPlaneInitialValue(landmarkId, initialValue);
}

void FactorGraphExternal::createOdometryNoiseModel(float *variance)
{
    gtsam::Vector6 odomVariance;
    odomVariance << variance[0], variance[1], variance[2], variance[3], variance[4], variance[5];
    factorGraphHandler.createOdometryNoiseModel(odomVariance);
}

void FactorGraphExternal::createOrientedPlaneNoiseModel(float *variance)
{
    gtsam::Vector3 lmVariance;
    lmVariance << variance[0], variance[1], variance[2];
    factorGraphHandler.createOrientedPlaneNoiseModel(lmVariance);
}

bool FactorGraphExternal::getPoseById(int poseId, double* pose)
{
    if(!factorGraphHandler.getResults().exists(gtsam::Symbol('x', poseId)))
        return false;

    auto matrix = factorGraphHandler.getResults().at<gtsam::Pose3>(gtsam::Symbol('x', poseId)).matrix();

    matrix.transposeInPlace();

    std::copy(  matrix.data(),
                matrix.data() + 16,
                pose);

    return true;
}

bool FactorGraphExternal::getPlanarLandmarkById(int landmarkId, double* plane)
{
    if(!factorGraphHandler.getResults().exists(gtsam::Symbol('l', landmarkId)))
        return false;

    auto matrix = factorGraphHandler.getResults().at<gtsam::OrientedPlane3>(gtsam::Symbol('l', landmarkId)).planeCoefficients();

    std::copy(  matrix.data(),
                matrix.data() + 4,
                plane);

    return true;
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