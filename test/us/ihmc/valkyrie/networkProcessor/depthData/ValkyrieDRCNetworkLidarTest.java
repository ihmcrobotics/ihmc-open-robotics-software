package us.ihmc.valkyrie.networkProcessor.depthData;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.DRCNetworkLidarTest;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieDRCNetworkLidarTest extends DRCNetworkLidarTest
{
   private DRCRobotModel robotModel = new ValkyrieRobotModel(false, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }
}
