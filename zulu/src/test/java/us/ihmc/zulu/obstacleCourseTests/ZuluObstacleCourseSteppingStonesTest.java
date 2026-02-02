package us.ihmc.zulu.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseSteppingStonesTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

// this test is flaky because toe off is not triggered properly - the toe off condition needs to be fixed
public class ZuluObstacleCourseSteppingStonesTest extends DRCObstacleCourseSteppingStonesTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }

   @Tag("humanoid-obstacle-slow")
   @Override
   @Test
   public void testWalkingOverEasySteppingStones()
   {
      super.testWalkingOverEasySteppingStones();
   }
}
