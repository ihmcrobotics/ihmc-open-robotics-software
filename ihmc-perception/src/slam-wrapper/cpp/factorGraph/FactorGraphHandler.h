#pragma once

#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

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

      void optimize();

      void optimizeISAM2(uint8_t numberOfUpdates);

      void clearISAM2();

      void setPoseInitialValue(int index, gtsam::Pose3 value);

      void setOrientedPlaneInitialValue(int landmarkId, gtsam::OrientedPlane3 value);

      const gtsam::Values& getResults() const {return result;};

      const gtsam::Values& getInitialValues() const {return initial;};

      const gtsam::NonlinearFactorGraph& getFactorGraph();

      void createOdometryNoiseModel(gtsam::Vector6 odomVariance);

      void createOrientedPlaneNoiseModel(gtsam::Vector3 lmVariances);

      void incrementPoseId();

      void SLAMTest();

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

};