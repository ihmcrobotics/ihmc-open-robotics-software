package us.ihmc.valkyrie;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryDoubleSupportTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class ValkyriePushInDoubleSupportTest extends DRCPushRecoveryDoubleSupportTest
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
}
