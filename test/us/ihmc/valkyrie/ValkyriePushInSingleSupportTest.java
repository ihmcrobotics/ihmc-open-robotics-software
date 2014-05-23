package us.ihmc.valkyrie;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCPushRecoverySingleSupportTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class ValkyriePushInSingleSupportTest extends DRCPushRecoverySingleSupportTest
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