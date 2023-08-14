package us.ihmc.atlas.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseRampFootstepSnapperTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasObstacleCourseRampFootstepSnapperTest extends DRCObstacleCourseRampFootstepSnapperTest
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

   @Tag("humanoid-obstacle-slow")
   @Override
   @Test
   public void testWalkingUpRampUsingSnapFootsteps()
   {
      super.testWalkingUpRampUsingSnapFootsteps();
   }
}
