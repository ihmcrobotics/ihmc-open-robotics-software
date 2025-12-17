package us.ihmc.openAlexander.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseStandingYawedTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AlexanderObstacleCourseStandingYawedTest extends DRCObstacleCourseStandingYawedTest
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

   @Tag("humanoid-flat-ground")
   @Override
   @Test
   public void testStandingYawed()
   {
      super.testStandingYawed();
   }

}
