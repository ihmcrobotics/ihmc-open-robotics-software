package us.ihmc.valkyrie.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationPushRecoveryTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieICPOptimizationPushRecoveryTest extends ICPOptimizationPushRecoveryTest
{
   protected DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
   }
}
