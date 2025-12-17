package us.ihmc.openAlexander.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseRampsTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AlexanderObstacleCourseRampsTest extends DRCObstacleCourseRampsTest
{
   private final DRCRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   @Tag("humanoid-obstacle")
   @Override
   @Test
   public void testWalkingDownRampWithMediumSteps()
   {
      super.testWalkingDownRampWithMediumSteps();
   }

   @Tag("humanoid-obstacle")
   @Override
   @Test
   public void testWalkingUpRampWithMediumSteps()
   {
      super.testWalkingUpRampWithMediumSteps();
   }

   @Tag("humanoid-obstacle-slow")
   @Override
   @Test
   public void testWalkingUpRampWithShortSteps()
   {
      super.testWalkingUpRampWithShortSteps();
   }

   @Tag("humanoid-obstacle-slow")
   @Override
   @Test
   public void testWalkingUpRampWithShortStepsALittleTooHigh()
   {
      super.testWalkingUpRampWithShortStepsALittleTooHigh();
   }

   @Tag("humanoid-obstacle-slow")
   @Override
   @Test
   public void testWalkingUpRampWithShortStepsALittleTooLow()
   {
      super.testWalkingUpRampWithShortStepsALittleTooLow();
   }

   @Override
   protected double getMaxRotationCorruption()
   {
      return Math.PI / 8.0;
   }

}
