package us.ihmc.valkyrie.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationPushRecoveryTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;

public class ValkyrieICPOptimizationPushRecoveryTest extends ICPOptimizationPushRecoveryTest
{
   protected DRCRobotModel getRobotModel()
   {
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new ValkyrieWalkingControllerParameters(getJointMap(), RobotTarget.SCS)
            {
               @Override
               public boolean useOptimizationBasedICPController()
               {
                  return true;
               }
            };
         }
      };

      return valkyrieRobotModel;
   }
}
