#pragma once

#include "FactorGraphHandler.h"

class FactorGraphExternal
{
   public:
      // Expects packed Pose3 as XYZYPR
      void addPriorPoseFactor(int index, double* pose);

      // Expects packed Pose3 as XYZYPR
      void addOdometryFactor(int poseId, double* odometry);

      // Expects packed Vector4
      void addOrientedPlaneFactor(int lmId, int poseIndex, float* lmMean);

      void optimize();

      void optimizeISAM2(uint8_t numberOfUpdates);

      void clearISAM2();

      // Expects packed Pose3
      void setPoseInitialValue(int index, double* value);

      // Expects packed OrientedPlane3
      void setOrientedPlaneInitialValue(int landmarkId, float* value);

      // Expects packed Vector6
      void createOdometryNoiseModel(float* odomVariance);

      // Expects packed Vector3
      void createOrientedPlaneNoiseModel(float* lmVariances);

      void getResultPoses(double* poses, uint32_t* poseIDs, uint32_t count);

      void getResultLandmarks(double* landmarks, uint32_t* landmarkIDs, uint32_t count);

      void addGenericProjectionFactor(float *point, int lmId, int poseIndex);

      void setPointLandmarkInitialValue(int landmarkId, float* value);

      // Expects 4x4 homogenous transform matrix as 16-double array
      void addOdometryFactorSE3(int poseId, double* odometry);

      // Expects 4x4 homogenous transform as 16-double array
      void setPoseInitialValueSE3(int index, double* value);

      // Add Prior Pose Factor with full 4x4 homogenous SE3 matrix
      void addPriorPoseFactorSE3(int poseId, double* pose);

      bool getPoseById(int poseId, double* pose);

      bool getPlanarLandmarkById(int poseId, double* plane);

      void printResults();

      void helloWorldTest();

      // void visualSLAMTest();


   private:
      FactorGraphHandler factorGraphHandler;
};