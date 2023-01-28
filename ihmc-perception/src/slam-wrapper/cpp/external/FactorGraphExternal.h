#pragma once

#include "FactorGraphHandler.h"

class FactorGraphExternal
{
    public:
        // Expects packed Pose3 as XYZYPR
        void addPriorPoseFactor(int index, float* pose);

        // Expects packed Pose3 as XYZYPR
        void addOdometryFactor(float* odometry, int poseId);

        // Expects 4x4 homogenous transform matrix as 16-float array
        void addOdometryFactorExtended(float *odometry, int poseId);

        // Expects packed Vector4
        void addOrientedPlaneFactor(float* lmMean, int lmId, int poseIndex);

        void optimize();

        void optimizeISAM2(uint8_t numberOfUpdates);

        void clearISAM2();

        // Expects packed Pose3
        void setPoseInitialValue(int index, float* value);

        // Expects 4x4 homogenous transform as 16-float array
        void setPoseInitialValueExtended(int index, float *value);

        // Expects packed OrientedPlane3
        void setOrientedPlaneInitialValue(int landmarkId, float* value);

        // Expects packed Vector6
        void createOdometryNoiseModel(float* odomVariance);

        // Expects packed Vector3
        void createOrientedPlaneNoiseModel(float* lmVariances);

        bool getPoseById(int poseId, double* pose);

        void printResults();

        void helloWorldTest();

    private:
        FactorGraphHandler factorGraphHandler;
};