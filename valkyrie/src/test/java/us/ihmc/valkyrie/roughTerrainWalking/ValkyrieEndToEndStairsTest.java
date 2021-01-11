package us.ihmc.valkyrie.roughTerrainWalking;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidEndToEndStairsTest;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieEndToEndStairsTest extends HumanoidEndToEndStairsTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
   }

   @Test
   @Override
   public void testUpStairsSlow(TestInfo testInfo) throws Exception
   {
      super.testUpStairsSlow(testInfo);
   }
}
