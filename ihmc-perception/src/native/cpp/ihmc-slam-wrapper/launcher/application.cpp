
#include "factor_graph_handler.h"

void SLAMTest(FactorGraphHandler& factorGraphHandler)
{
   using namespace gtsam;

   Pose3 init_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
   factorGraphHandler.addPriorPoseFactor(1, init_pose);

   Pose3 odometry(Rot3::Ypr(0.0, 0.0, 0.0), Point3(1.0, 0.0, 0.0));
   factorGraphHandler.addOdometryFactor(odometry, 2);

   factorGraphHandler.optimizeISAM2(3);

}

int main()
{
    FactorGraphHandler factorGraphHandler;

    factorGraphHandler.getResults().print();

    
}