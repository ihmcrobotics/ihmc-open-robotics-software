package us.ihmc.valkyrie.roughTerrainWalking;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidEndToEndStairsTest;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieEndToEndStairsTest extends HumanoidEndToEndStairsTest
{
   private boolean useVal2Scale = false;

   @Override
   public DRCRobotModel getRobotModel()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      if (useVal2Scale)
      {
         robotModel.setModelSizeScale(0.925170);
         robotModel.setModelMassScale(0.925170);
      }
      return robotModel;
   }

   @Test
   public void testUpStairsSlow(TestInfo testInfo) throws Exception
   {
      useVal2Scale = false;
      Random random = new Random(53415);
      testStairs(testInfo, true, true, 0.6, 0.25, 0.0, createFootstepCorruptor(random, 0.025, 0.10, 0.05, 0.2, 0.2, 0.2));
   }

   @Test
   public void testDownStairsSlow(TestInfo testInfo) throws Exception
   {
      useVal2Scale = false;
      Random random = new Random(53415);
      testStairs(testInfo, true, false, 0.9, 0.25, 0.0, createFootstepCorruptor(random, 0.025, 0.10, 0.05, 0.2, 0.2, 0.2));
   }

   @Test
   public void testUpStairs(TestInfo testInfo) throws Exception
   {
      useVal2Scale = false;
      testStairs(testInfo, false, true, 1.0, 0.25, 0.0);
   }

   @Test
   public void testDownStairs(TestInfo testInfo) throws Exception
   {
      useVal2Scale = false;
      Random random = new Random(53415);
      testStairs(testInfo, false, false, 1.0, 0.35, 0.0, createFootstepCorruptor(random, 0.025, 0.10, 0.05, 0.2, 0.2, 0.2));
   }

   @Test
   public void testUpStairsSlowVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      Random random = new Random(53415);
      testStairs(testInfo, true, true, 0.6, 0.25, 0.0, createFootstepCorruptor(random, 0.02, 0.05, 0.05, 0.2, 0.2, 0.2));
   }

   @Test
   public void testDownStairsSlowVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      Random random = new Random(53415);
      testStairs(testInfo, true, false, 1.0, 0.25, 0.0, createFootstepCorruptor(random, 0.02, 0.05, 0.05, 0.1, 0.2, 0.2));
   }

   @Test
   public void testUpStairsVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      testStairs(testInfo, false, true, 1.2, 0.25, 0.0);
   }

   @Test
   public void testDownStairsVal2Scale(TestInfo testInfo) throws Exception
   {
      useVal2Scale = true;
      Random random = new Random(53415);
      testStairs(testInfo, false, false, 1.0, 0.35, 0.0, createFootstepCorruptor(random, 0.02, 0.05, 0.05, 0.2, 0.2, 0.2));
   }
}
