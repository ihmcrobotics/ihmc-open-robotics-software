
#include "factor_graph_handler.h"
#include "factor_graph_external.h"

void SLAMTest(FactorGraphHandler& factorGraphHandler)
{
   using namespace gtsam;

   Pose3 init_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
   factorGraphHandler.addPriorPoseFactor(1, init_pose);

   Pose3 odometry(Rot3::Ypr(0.0, 0.0, 0.0), Point3(1.0, 0.0, 0.0));
   factorGraphHandler.addOdometryFactor(odometry, 2);

   factorGraphHandler.setPoseInitialValue(1, init_pose);
   factorGraphHandler.setPoseInitialValue(2, odometry);

   factorGraphHandler.optimizeISAM2(3);

    factorGraphHandler.getResults().print();


}

void ExternalSLAMTest(FactorGraphExternal& factorGraphExternal)
{
   using namespace gtsam;

   

   float initial_pose_value[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   factorGraphExternal.addPriorPoseFactor(1, initial_pose_value);

   float odometry_value[] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
   factorGraphExternal.addOdometryFactor(odometry_value, 2);

   factorGraphExternal.setPoseInitialValue(1, initial_pose_value);
   factorGraphExternal.setPoseInitialValue(2, odometry_value);

   factorGraphExternal.optimizeISAM2(3);

    factorGraphExternal.printResults();


}

int main()
{
    FactorGraphExternal factorGraphExternal;


    ExternalSLAMTest(factorGraphExternal);

    printf("Welcome To Factor Graphs in Java World!\n");

    
}