#include "factor_graph_external.h"

void FactorGraphExternal::addPriorPoseFactor(int index, float *pose)
{
}

void FactorGraphExternal::addOdometryFactor(float *odometry, int poseId)
{
}

void FactorGraphExternal::addOrientedPlaneFactor(float *lmMean, int lmId, int poseIndex)
{
}

void FactorGraphExternal::optimize()
{
}

void FactorGraphExternal::optimizeISAM2(uint8_t numberOfUpdates)
{
}

void FactorGraphExternal::clearISAM2()
{
}

void FactorGraphExternal::setPoseInitialValue_Pose3(int index, float *value)
{
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