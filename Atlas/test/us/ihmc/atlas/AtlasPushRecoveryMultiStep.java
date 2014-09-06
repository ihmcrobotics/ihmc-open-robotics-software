package us.ihmc.atlas;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryMultiStep;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class AtlasPushRecoveryMultiStep extends DRCPushRecoveryMultiStep
{
   @Override
   public DRCRobotModel getRobotModel() 
   {
      return new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, false, false);
   }

   @Override
   public String getSimpleRobotName() 
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   protected void setForwardPushParameters()
   {
      forceMagnitude = 500.0;
      forceDuration = 0.2;
   }

   @Override
   protected void setBackwardPushParameters()
   {
      forceMagnitude = -500.0;
      forceDuration = 0.2;
      
   }
}
