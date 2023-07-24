package us.ihmc.atlas.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseRampsTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasObstacleCourseRampsTest extends DRCObstacleCourseRampsTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
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
