package us.ihmc.valkyrie;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryMultiStep;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class ValkyriePushRecoveryMultiStep extends DRCPushRecoveryMultiStep
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(false, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
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
