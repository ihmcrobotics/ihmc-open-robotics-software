#pragma once

#include "FactorGraphHandler.h"

class FactorGraphExternal
{
    public:
        // Expects packed Pose3 as XYZYPR
        void addPriorPoseFactor(int index, float* pose);

        // Expects packed Pose3 as XYZYPR
        void addOdometryFactor(float* odometry, int poseId);

        // Expects 4x4 homogenous transform matrix to insert Pose3 factor
        void addOdometryFactorExtended(double *odometry, int poseId);

        // Expects packed Vector4
        void addOrientedPlaneFactor(float* lmMean, int lmId, int poseIndex);

        void optimize();

        void optimizeISAM2(uint8_t numberOfUpdates);

        void clearISAM2();

        // Expects packed Pose3
        void setPoseInitialValue(int index, float* value);

        // Expects 4x4 homogenous transform matrix as initial value for Pose3
        void setPoseInitialValueExtended(int index, float *value);

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

        void printResults();

        void helloWorldTest();

        void visualSLAMTest();


    private:
        FactorGraphHandler factorGraphHandler;
};