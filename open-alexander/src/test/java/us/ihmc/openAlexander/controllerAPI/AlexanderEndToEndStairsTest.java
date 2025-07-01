package us.ihmc.openAlexander.controllerAPI;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;
import us.ihmc.openAlexander.OpenAlexanderVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.roughTerrainWalking.HumanoidEndToEndStairsTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class AlexanderEndToEndStairsTest extends HumanoidEndToEndStairsTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(OpenAlexanderVersion.V1_FULL_ROBOT);
   }

   @Disabled
   @Test
   public void testUpStairsSlow(TestInfo testInfo) throws Exception
   {
      testStairs(testInfo, true, true, 0.6, 0.25, 0.0);
   }

   @Disabled
   @Test
   public void testDownStairsSlow(TestInfo testInfo) throws Exception
   {
      testStairs(testInfo, true, false, 0.9, 0.25, 0.0);
   }

   @Disabled
   @Test
   public void testUpStairs(TestInfo testInfo) throws Exception
   {
      testStairs(testInfo, false, true, 0.9, 0.25, 0.0);
   }

   @Disabled
   @Test
   public void testDownStairs(TestInfo testInfo) throws Exception
   {
      testStairs(testInfo, false, false, 1.0, 0.35, 0.0);
   }

   /**
    * Test for comparison against real robot data \\Gideon\LogData\Atlas\20210624_AtlasDemoFilming\20210624_140852_AtlasStairsPipeBombDisposalBestRunYet
    */
   @Disabled
   @Test
   public void testUpStairsSlowFBDemo(TestInfo testInfo) throws Exception
   {
      setStepHeight(6.75 * 0.0254);
      setStepLength(0.285);
      setUseExperimentalPhysicsEngine(true);
      setNumberOfSteps(3);
      testStairs(testInfo, true, true, 2.0, 0.8, 0.0);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ALEXANDER);
   }
}
