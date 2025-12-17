package us.ihmc.openAlexander.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.roughTerrainWalking.HumanoidEndToEndStairsTest;

@Tag("humanoid-stairs-slow")
public class AlexanderEndToEndStairsTest extends HumanoidEndToEndStairsTest
{
   private ZuluVersion selectedVersion = ZuluVersion.V1_FULL_ROBOT;

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(selectedVersion);
   }

   @Test
   public void testUpStairsSlow(TestInfo testInfo) throws Exception
   {
      testStairs(testInfo, true, true, 0.6, 0.25, -0.05);
   }

   @Test
   public void testDownStairsSlow(TestInfo testInfo) throws Exception
   {
      testStairs(testInfo, true, false, 0.9, 0.25, 0.0);
   }

   @Test
   public void testUpStairs(TestInfo testInfo) throws Exception
   {
      testStairs(testInfo, false, true, 0.9, 0.25, 0.0);
   }

   @Test
   public void testDownStairs(TestInfo testInfo) throws Exception
   {
      testStairs(testInfo, false, false, 1.0, 0.35, 0.0);
   }

   @Test
   public void testSpecialStairs() throws Exception
   {
      testSpecialStairs(true, 0.8, 0.5);
   }

}
