#pragma once

#include "factor_graph_handler.h"

class FactorGraphExternal
{
    public:
        // Expects packed Pose3
        void addPriorPoseFactor(int index, float* pose);

        // Expects packed Pose3
        void addOdometryFactor(float* odometry, int poseId);

        // Expects packed Vector4
        void addOrientedPlaneFactor(float* lmMean, int lmId, int poseIndex);

        void optimize();

        void optimizeISAM2(uint8_t numberOfUpdates);

        void clearISAM2();

        // Expects packed Pose3
        void setPoseInitialValue_Pose3(int index, float* value);

        // Expects packed OrientedPlane3
        void setOrientedPlaneInitialValue(int landmarkId, float* value);

        // Expects packed Vector6
        void createOdometryNoiseModel(float* odomVariance);

        // Expects packed Vector3
        void createOrientedPlaneNoiseModel(float* lmVariances);

        void helloWorldTest();

    private:
        FactorGraphHandler factorGraphHandler;
};