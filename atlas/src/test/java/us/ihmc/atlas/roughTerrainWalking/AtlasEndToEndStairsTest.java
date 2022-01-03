package us.ihmc.atlas.roughTerrainWalking;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.roughTerrainWalking.HumanoidEndToEndStairsTest;

public class AtlasEndToEndStairsTest extends HumanoidEndToEndStairsTest
{
   private AtlasRobotVersion selectedVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(selectedVersion);
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testUpStairsSlow(TestInfo testInfo) throws Exception
   {
      selectedVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
      testStairs(testInfo, true, true, 0.6, 0.25, 0.0);
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testDownStairsSlow(TestInfo testInfo) throws Exception
   {
      selectedVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
      testStairs(testInfo, true, false, 0.9, 0.25, 0.0);
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testUpStairs(TestInfo testInfo) throws Exception
   {
      selectedVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
      testStairs(testInfo, false, true, 0.9, 0.25, 0.04);
   }

   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testDownStairs(TestInfo testInfo) throws Exception
   {
      selectedVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
      testStairs(testInfo, false, false, 1.0, 0.35, 0.0);
   }

   /**
    * Test for comparison against real robot data \\Gideon\LogData\Atlas\20210624_AtlasDemoFilming\20210624_140852_AtlasStairsPipeBombDisposalBestRunYet
    */
   @Disabled
   @Test
   @Tag("humanoid-rough-terrain-slow")
   public void testUpStairsSlowFBDemo(TestInfo testInfo) throws Exception
   {
      selectedVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
      setStepHeight(6.75 * 0.0254);
      setStepLength(0.285);
      setUseExperimentalPhysicsEngine(true);
      setNumberOfSteps(3);
      testStairs(testInfo, true, true, 2.0, 0.8, 0.0);
   }
}
