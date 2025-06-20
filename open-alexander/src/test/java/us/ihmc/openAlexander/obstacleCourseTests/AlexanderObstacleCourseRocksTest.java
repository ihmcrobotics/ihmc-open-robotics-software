package us.ihmc.openAlexander.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.OpenAlexanderVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseRocksTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AlexanderObstacleCourseRocksTest extends DRCObstacleCourseRocksTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(OpenAlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   @Tag("humanoid-obstacle")
   @Test
   public void testWalkingOntoRocks()
   {
      super.testWalkingOntoRocks();
   }
}
