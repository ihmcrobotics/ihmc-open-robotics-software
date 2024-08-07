package us.ihmc.atlas.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseSteppingStonesTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

// this test is flaky because toe off is not triggered properly - the toe off condition needs to be fixed
public class AtlasObstacleCourseSteppingStonesTest extends DRCObstacleCourseSteppingStonesTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }

   @Tag("humanoid-obstacle-slow")
   @Override
   @Test
   public void testWalkingOverEasySteppingStones()
   {
      super.testWalkingOverEasySteppingStones();
   }
}
