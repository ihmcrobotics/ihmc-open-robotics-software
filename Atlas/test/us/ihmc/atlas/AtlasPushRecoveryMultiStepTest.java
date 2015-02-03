package us.ihmc.atlas;

import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryMultiStepTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.CustomJob;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.CustomJobType;

@CustomJob(job = CustomJobType.PushRecovery)
public class AtlasPushRecoveryMultiStepTest extends DRCPushRecoveryMultiStepTest
{
   @Override
   public DRCRobotModel getRobotModel() 
   {
      return new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);
   }

   @Override
   public String getSimpleRobotName() 
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   protected void setForwardPushParameters()
   {
      forceMagnitude = 400.0;
      forceDuration = 0.2;
   }

   @Override
   protected void setBackwardPushParameters()
   {
      forceMagnitude = -400.0;
      forceDuration = 0.2;
      
   }
}
