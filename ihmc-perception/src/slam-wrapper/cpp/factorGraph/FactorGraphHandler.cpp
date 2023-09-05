#include "FactorGraphHandler.h"

FactorGraphHandler::FactorGraphHandler()
{
   /* Set ISAM2 parameters here. */
   parameters.relinearizeThreshold = 0.01;
   parameters.relinearizeSkip = 1;
   this->isam = gtsam::ISAM2(parameters);

   // gtsam::Vector6 odomVariance;
   // odomVariance << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2;
   // createOdometryNoiseModel(odomVariance);

   // gtsam::Vector3 lmVariance;
   // lmVariance << 1e-2, 1e-2, 1e-2;
   // createOrientedPlaneNoiseModel(lmVariance);

   gtsam::Vector6 priorVariance;
   priorVariance << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
   priorNoise = gtsam::noiseModel::Diagonal::Variances(priorVariance);

   gtsam::Vector6 priorVariance2;
   priorVariance2 << 1e2, 1e2, 1e2, 1e2, 1e2, 1e2;
   priorNoise2 = gtsam::noiseModel::Diagonal::Variances(priorVariance2);
}

void FactorGraphHandler::createOdometryNoiseModel(gtsam::Vector6 odomVariance)
{
   odometryNoise = gtsam::noiseModel::Diagonal::Variances(odomVariance);
}

void FactorGraphHandler::createOrientedPlaneNoiseModel(gtsam::Vector3 lmVariances)
{
   orientedPlaneNoise = gtsam::noiseModel::Diagonal::Variances(lmVariances);
}

void FactorGraphHandler::addPriorPoseFactor(int index, gtsam::Pose3 mean)
{
   // printf("Prior Pose Factor: x%d\n", index); fflush(stdout);
   graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', index), mean, priorNoise));
}

void FactorGraphHandler::addOdometryFactor(gtsam::Pose3 odometry, int poseId)
{
   // printf("Odometry Factor: x%d -> x%d\n", poseId - 1, poseId); fflush(stdout);
   graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', poseId - 1), gtsam::Symbol('x', poseId), odometry, odometryNoise));
   poseId++;
}

void FactorGraphHandler::addOrientedPlaneFactor(gtsam::Vector4 lmMean, int lmId, int poseIndex)
{
   // printf("Plane Factor: x%d -> l%d\n", poseIndex, lmId); fflush(stdout);
   graph.add(gtsam::OrientedPlane3Factor(lmMean, orientedPlaneNoise, gtsam::Symbol('x', poseIndex), gtsam::Symbol('l', lmId)));
}

void FactorGraphHandler::setPoseInitialValue(int index, gtsam::Pose3 value)
{
   // printf("Pose Initial Value: x%d\n", index); fflush(stdout);
   if (structure.find('x' + std::to_string(index)) == structure.end())
   {
      structure.insert('x' + std::to_string(index));
      initial.insert(gtsam::Symbol('x', index), value);
   }
}

void FactorGraphHandler::setOrientedPlaneInitialValue(int landmarkId, gtsam::OrientedPlane3 value)
{
   // printf("Plane Initial Value: l%d\n", landmarkId); fflush(stdout);
   if (!initial.exists(gtsam::Symbol('l', landmarkId)) && structure.find('l' + std::to_string(landmarkId)) == structure.end())
   {
      structure.insert('l' + std::to_string(landmarkId));
      initial.insert(gtsam::Symbol('l', landmarkId), value);
   }
}

void FactorGraphHandler::optimize()
{
   result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
}

void FactorGraphHandler::optimizeISAM2(uint8_t numberOfUpdates)
{
   isam.update(graph, initial);
   for (uint8_t i = 1; i < numberOfUpdates; i++)
   {
      isam.update();
   }
   result = isam.calculateEstimate();
}

void FactorGraphHandler::clearISAM2()
{
   initial.clear();
   graph.resize(0);
}

const gtsam::NonlinearFactorGraph& FactorGraphHandler::getFactorGraph()
{
   return graph;
}

void FactorGraphHandler::SLAMTest()
{
   using namespace gtsam;

   int currentPoseId = 1;

   Pose3 init_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
   addPriorPoseFactor(currentPoseId, Pose3::Identity());
   setPoseInitialValue(currentPoseId, Pose3::Identity());

   addOrientedPlaneFactor(Vector4(1, 0, 0, -3), 0, currentPoseId);
   setOrientedPlaneInitialValue(0, gtsam::OrientedPlane3(Vector4(0.8, 0.1, 0.1, -2.9)));

   addOrientedPlaneFactor(Vector4(0, 0, 1, -3), 1, currentPoseId);
   setOrientedPlaneInitialValue(1, gtsam::OrientedPlane3(Vector4(0.1, 0.04, 1.1, -2.8)));

   Pose3 odometry(Rot3::Ypr(0.0, 0.0, 0.0), Point3(1.0, 0.0, 0.0));
   addOdometryFactor(odometry, 1);
   setPoseInitialValue(currentPoseId, odometry);

   addOrientedPlaneFactor(Vector4(1, 0, 0, -2), 0, currentPoseId);
   setOrientedPlaneInitialValue(0, gtsam::OrientedPlane3(Vector4(0.8, 0.1, 0.1, -2.1)));

   addOrientedPlaneFactor(Vector4(0, 0, 1, -3), 1, currentPoseId);
   setOrientedPlaneInitialValue(1, gtsam::OrientedPlane3(Vector4(0.1, 0.04, 1.1, -2.2)));

   optimize();

   result.print("Result Planes");
}