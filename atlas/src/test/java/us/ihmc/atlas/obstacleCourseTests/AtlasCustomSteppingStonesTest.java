package us.ihmc.atlas.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.AvatarCustomSteppingStonesTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasCustomSteppingStonesTest extends AvatarCustomSteppingStonesTest
{
   @Tag("humanoid-obstacle-2")
   @Test
   public void testToeOffTakingBigStepsUp(TestInfo testInfo)
   {
      /**
       * TODO remove as it assimilates
       * {@link us.ihmc.atlas.roughTerrainWalking.AtlasEndToEndStairsTest#testUpStairs(TestInfo)}
       */
      changeWalkingParameters(0.3, 1.05);
      setNumberOfSteps(4);
      super.testTakingStep(testInfo, 0.24);
   }

   @Tag("humanoid-obstacle-2")
   @Test
   public void testTakingBigStepUpAndStopping(TestInfo testInfo)
   { // NOTE this passes but violates knee joint limits
      setTakeSquareUpStep(false);
      super.testTakingStep(testInfo, 0.4);
   }

   @Tag("humanoid-obstacle-2")
   @Test
   public void testTakingBigStepUpAndStoppingThenSquaringUp(TestInfo testInfo)
   { // NOTE this passes but violates knee joint limits
      changeWalkingParameters(0.3, 0.95);
      setNumberOfSteps(4);
      super.testTakingStepOneFootAtATime(testInfo, 0.22);
   }

   @Tag("humanoid-obstacle-2")
   @Test
   public void testTakingBigStepUpAndSquaringUp(TestInfo testInfo)
   { // NOTE this passes but the shins collide with stair step
      super.testTakingStep(testInfo, 0.4);
   }

   @Tag("humanoid-obstacle-2")
   @Test
   public void testToeOffTakingBigSideStepUp(TestInfo testInfo)
   {
      changeWalkingParameters(0.25, 0.8);
      super.testTakingStep(testInfo, 0.35, -0.25);
   }

   @Override
   public double getStepLength()
   {
      return 0.4;
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
