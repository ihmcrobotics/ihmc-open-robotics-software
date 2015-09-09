package us.ihmc.atlas;

import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryMultiStepTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;

@DeployableTestClass(targets = {TestPlanTarget.Fast, TestPlanTarget.VideoA})
public class AtlasPushRecoveryMultiStepTest extends DRCPushRecoveryMultiStepTest
{
   @Override
   public DRCRobotModel getRobotModel() 
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName() 
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   protected void setForwardPushParameters()
   {
      forceMagnitude = 750.0;
      forceDuration = 0.2;
   }

   @Override
   protected void setBackwardPushParameters()
   {
      forceMagnitude = -700.0;
      forceDuration = 0.2;
   }
}
