package us.ihmc.zulu.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.AvatarCustomSteppingStonesTest;
import us.ihmc.zulu.parameters.controller.ZuluWalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class ZuluCustomSteppingStonesTest extends AvatarCustomSteppingStonesTest
{
   @Tag("humanoid-obstacle-2")
   @Test
   public void testToeOffTakingBigStepsUp(TestInfo testInfo)
   {
      /**
       * TODO remove as it assimilates
       * {@link us.ihmc.atlas.roughTerrainWalking.AtlasEndToEndStairsTest#testUpStairs(TestInfo)}
       */
      changeWalkingParameters(0.3, 0.8);
      setNumberOfSteps(4);
      super.testTakingStep(testInfo, 0.24);
   }

   @Tag("humanoid-obstacle-2")
   @Test
   public void testTakingBigStepUpAndStopping(TestInfo testInfo)
   { // NOTE this passes but violates knee joint limits
      setTakeSquareUpStep(false);
      super.testTakingStep(testInfo, 0.35);
   }

   @Tag("humanoid-obstacle-2")
   @Test
   public void testTakingBigStepUpAndStoppingThenSquaringUp(TestInfo testInfo)
   { // NOTE this passes but violates knee joint limits
      changeWalkingParameters(0.3, 0.9);
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
      changeWalkingParameters(0.65, 0.75);
      super.testTakingStep(testInfo, 0.35, -0.25, -0.05);
   }

   @Override
   public double getStepLength()
   {
      return 0.4;
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new ZuluWalkingControllerParameters(getRobotVersion(), getTarget(), getJointMap(), getPhysicalProperties())
            {
               @Override
               public double nominalHeightAboveAnkle()
               {
                  return 0.91;
               }

               @Override
               public double maximumHeightAboveAnkle()
               {
                  return 0.93;
               }
            };
         }
      };
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }
}
