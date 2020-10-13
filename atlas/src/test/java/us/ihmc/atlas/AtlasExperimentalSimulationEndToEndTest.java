package us.ihmc.atlas;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.HumanoidExperimentalSimulationEndToEndTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;

public class AtlasExperimentalSimulationEndToEndTest extends HumanoidExperimentalSimulationEndToEndTest
{
   private static final AtlasRobotVersion VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(VERSION, RobotTarget.SCS);
   }

   @Test
   @Override
   public void testStanding(TestInfo testInfo) throws Exception
   {
      super.testStanding(testInfo);
   }

   @Test
   @Override
   public void testZeroTorque(TestInfo testInfo) throws Exception
   {
      super.testZeroTorque(testInfo);
   }
}
