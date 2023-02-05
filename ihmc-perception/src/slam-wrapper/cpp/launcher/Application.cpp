#include "FactorGraphHandler.h"
#include "FactorGraphExternal.h"

void SLAMTest(FactorGraphHandler& factorGraphHandler)
{
   using namespace gtsam;

   Pose3 initPose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
   factorGraphHandler.addPriorPoseFactor(1, initPose);

   Pose3 odometry(Rot3::Ypr(0.0, 0.0, 0.0), Point3(1.0, 0.0, 0.0));
   factorGraphHandler.addOdometryFactor(odometry, 2);

   factorGraphHandler.setPoseInitialValue(1, initPose);
   factorGraphHandler.setPoseInitialValue(2, odometry);

   factorGraphHandler.optimizeISAM2(3);

   factorGraphHandler.getResults().print();
}

void ExternalSLAMTest(FactorGraphExternal& factorGraphExternal)
{
   using namespace gtsam;

   float initialPoseValue[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   factorGraphExternal.addPriorPoseFactor(1, initialPoseValue);

   float odometryValue[] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
   factorGraphExternal.addOdometryFactor(2, odometryValue);

   factorGraphExternal.setPoseInitialValue(1, initialPoseValue);
   factorGraphExternal.setPoseInitialValue(2, odometryValue);

   factorGraphExternal.optimizeISAM2(3);

   factorGraphExternal.printResults();
}

int main()
{
   FactorGraphExternal factorGraphExternal;

   ExternalSLAMTest(factorGraphExternal);

   printf("Welcome To Factor Graphs in Java World!\n");
}