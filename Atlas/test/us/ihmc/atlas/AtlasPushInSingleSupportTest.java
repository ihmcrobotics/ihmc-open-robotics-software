package us.ihmc.atlas;


import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCPushRecoverySingleSupportTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class AtlasPushInSingleSupportTest extends DRCPushRecoverySingleSupportTest
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
}
