#include "FactorGraphExternal.h"

void FactorGraphExternal::addPriorPoseFactor(int index, double* pose)
{
   using namespace gtsam;
   Pose3 initPose(Rot3::Ypr(pose[0], pose[1], pose[2]), Point3(pose[3], pose[4], pose[5]));
   factorGraphHandler.addPriorPoseFactor(index, initPose);
}

void FactorGraphExternal::addPriorPoseFactorSE3(int poseId, double* pose)
{
   using namespace gtsam;
   Eigen::Matrix4d eigenMatrixSE3 = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >(pose);

   Pose3 poseValue(eigenMatrixSE3);
   factorGraphHandler.addPriorPoseFactor(poseId, poseValue);
}

void FactorGraphExternal::addOdometryFactor(int poseId, double* odometry)
{
   using namespace gtsam;
   Pose3 odometryValue(Rot3::Ypr(odometry[0], odometry[1], odometry[2]), Point3(odometry[3], odometry[4], odometry[5]));
   factorGraphHandler.addOdometryFactor(odometryValue, poseId);
}

void FactorGraphExternal::addOdometryFactorSE3(int poseId, double* odometry)
{
   using namespace gtsam;
   Eigen::Matrix4d eigenMatrixSE3 = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >(odometry);

   Pose3 odometryValue(eigenMatrixSE3);
   factorGraphHandler.addOdometryFactor(odometryValue, poseId);
}


void FactorGraphExternal::addGenericProjectionFactor(float *point, int lmId, int poseIndex)
{
   // printf("addGenericProjectionFactor(x%d -> p%d)\n", poseIndex, lmId);
   factorGraphHandler.addGenericProjectionFactor(gtsam::Point2(point[0], point[1]), lmId, poseIndex);
}

void FactorGraphExternal::setPointLandmarkInitialValue(int landmarkId, float* value)
{
   // printf("setPointLandmarkInitialValue(%d)\n", landmarkId);
   factorGraphHandler.setPointLandmarkInitialValue(landmarkId, {value[0], value[1], value[2]});
}


void FactorGraphExternal::addOrientedPlaneFactor(int landmarkId, int poseId, float* landmarkMean)
{
   using namespace gtsam;
   Vector4 planeValue(landmarkMean[0], landmarkMean[1], landmarkMean[2], landmarkMean[3]);
   factorGraphHandler.addOrientedPlaneFactor(planeValue, landmarkId, poseId);
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

void FactorGraphExternal::setPoseInitialValue(int index, double* value)
{
   using namespace gtsam;
   Pose3 initialValue(Rot3::Ypr(value[0], value[1], value[2]), Point3(value[3], value[4], value[5]));
   factorGraphHandler.setPoseInitialValue(index, initialValue);
}

void FactorGraphExternal::setPoseInitialValueSE3(int index, double* value)
{
   // printf("setPoseInitialValueSE3(%d)\n", index); fflush(stdout);
   using namespace gtsam;
   Eigen::Matrix4d eigenMatrixSE3 = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >(value);

   // std::cout << "Set Pose Initial Extended: " << std::endl << M << std::endl;

   Pose3 initialValue(eigenMatrixSE3);
   factorGraphHandler.setPoseInitialValue(index, initialValue);
}

void FactorGraphExternal::setOrientedPlaneInitialValue(int landmarkId, float* value)
{
   using namespace gtsam;
   OrientedPlane3 initialValue(value[0], value[1], value[2], value[3]);
   factorGraphHandler.setOrientedPlaneInitialValue(landmarkId, initialValue);
}

void FactorGraphExternal::createOdometryNoiseModel(float* variance)
{
   gtsam::Vector6 odomVariance;
   odomVariance << variance[0], variance[1], variance[2], variance[3], variance[4], variance[5];
   factorGraphHandler.createOdometryNoiseModel(odomVariance);
}

void FactorGraphExternal::createOrientedPlaneNoiseModel(float* variance)
{
   gtsam::Vector3 lmVariance;
   lmVariance << variance[0], variance[1], variance[2];
   factorGraphHandler.createOrientedPlaneNoiseModel(lmVariance);
}


void FactorGraphExternal::getResultPoses(double* poses, uint32_t* poseIDs, uint32_t count)
{
   // printf("getResultPoses()\n");
   for(uint32_t i = 0; i<count; i++)
   {
      auto matrix = factorGraphHandler.getResults().at<gtsam::Pose3>(gtsam::Symbol('x', poseIDs[i])).matrix();

      matrix.transposeInPlace();

      // std::cout << "GetResultPoses" << std::endl << matrix << std::endl;
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

bool FactorGraphExternal::getPoseById(int poseId, double* pose)
{
   if (!factorGraphHandler.getResults().exists(gtsam::Symbol('x', poseId)))
      return false;

   auto matrix = factorGraphHandler.getResults().at<gtsam::Pose3>(gtsam::Symbol('x', poseId)).matrix();

   matrix.transposeInPlace();

   std::copy(matrix.data(), matrix.data() + 16, pose);

   return true;
}

bool FactorGraphExternal::getPlanarLandmarkById(int landmarkId, double* plane)
{
   if (!factorGraphHandler.getResults().exists(gtsam::Symbol('l', landmarkId)))
      return false;

   auto matrix = factorGraphHandler.getResults().at<gtsam::OrientedPlane3>(gtsam::Symbol('l', landmarkId)).planeCoefficients();

   std::copy(matrix.data(), matrix.data() + 4, plane);

   return true;
}

void FactorGraphExternal::printResults()
{
   factorGraphHandler.getResults().print();
}

void FactorGraphExternal::helloWorldTest()
{
   // std::cout << "Hello from native code" << std::endl;

   for (int i = 0; i < 5; i++)
   {
   //    std::cout << "Hello " << i << std::endl;
   }
}

// void FactorGraphExternal::visualSLAMTest()
// {
//    factorGraphHandler.VisualSLAMTest();
// }
