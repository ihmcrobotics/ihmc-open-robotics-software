package us.ihmc.openAlexander.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseSteppingStonesTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

// this test is flaky because toe off is not triggered properly - the toe off condition needs to be fixed
public class AlexanderObstacleCourseSteppingStonesTest extends DRCObstacleCourseSteppingStonesTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   @Tag("humanoid-obstacle-slow")
   @Override
   @Test
   public void testWalkingOverEasySteppingStones()
   {
      super.testWalkingOverEasySteppingStones();
   }
}
