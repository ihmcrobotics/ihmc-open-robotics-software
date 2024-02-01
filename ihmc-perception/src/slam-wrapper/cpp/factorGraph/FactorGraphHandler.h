#pragma once

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/OrientedPlane3Factor.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/SimpleCamera.h>

#include <boost/bind.hpp>
#include <boost/assign/std/vector.hpp>

#include <unordered_set>

using namespace boost::assign;

class FactorGraphHandler
{
   public:
      int getPoseId() const;

      FactorGraphHandler();

      //      void getPoses(std::vector<RigidBodyTransform>& poses);

      void addPriorPoseFactor(int index, gtsam::Pose3 mean);

      void addOdometryFactor(gtsam::Pose3 odometry, int poseId);

      void addOrientedPlaneFactor(gtsam::Vector4 lmMean, int lmId, int poseIndex);

      void addGenericProjectionFactor(gtsam::Point2 point, int lmId, int poseIndex);

      void optimize();

      void optimizeISAM2(uint8_t numberOfUpdates);

      void clearISAM2();

      void setPointLandmarkInitialValue(int landmarkId, gtsam::Point3 value);

      void setPoseInitialValue(int index, gtsam::Pose3 value);

      void setOrientedPlaneInitialValue(int landmarkId, gtsam::OrientedPlane3 value);

      const gtsam::Values& getResults() const {return result;};

      const gtsam::Values& getInitialValues() const {return initial;};

      const gtsam::NonlinearFactorGraph& getFactorGraph();

      void createOdometryNoiseModel(gtsam::Vector6 odomVariance);

      void createOrientedPlaneNoiseModel(gtsam::Vector3 lmVariances);

      void incrementPoseId();

      void SLAMTest();

      void VisualSLAMTest();

   private:
      gtsam::ISAM2Params parameters;

      gtsam::ISAM2 isam;
      std::unordered_set<std::string> structure;
      gtsam::Values initial, result;
      gtsam::NonlinearFactorGraph graph;
      gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
      gtsam::noiseModel::Diagonal::shared_ptr priorNoise2;
      gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
      gtsam::noiseModel::Diagonal::shared_ptr orientedPlaneNoise;

      gtsam::noiseModel::Diagonal::shared_ptr pointLandmarkNoise;
      
      
      gtsam::Cal3_S2::shared_ptr K;

};