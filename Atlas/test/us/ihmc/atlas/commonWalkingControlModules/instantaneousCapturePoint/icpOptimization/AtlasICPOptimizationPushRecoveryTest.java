package us.ihmc.atlas.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationPushRecoveryTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasICPOptimizationPushRecoveryTest extends ICPOptimizationPushRecoveryTest
{
   protected DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   }

   public static void main(String[] args)
   {
      AtlasICPOptimizationPushRecoveryTest test = new AtlasICPOptimizationPushRecoveryTest();
      try
      {
         test.testPushICPOptimizationLongForwardPushInSwing();
      }
      catch(SimulationExceededMaximumTimeException e)
      {

      }
   }
}
