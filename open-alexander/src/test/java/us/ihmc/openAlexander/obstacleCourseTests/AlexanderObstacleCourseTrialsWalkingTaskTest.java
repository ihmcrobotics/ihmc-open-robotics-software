package us.ihmc.openAlexander.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseTrialsWalkingTaskTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-obstacle-slow")
public class AlexanderObstacleCourseTrialsWalkingTaskTest extends DRCObstacleCourseTrialsWalkingTaskTest
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

   @Test
   @Override
   public void testStepOnAndOffCinderBlocks()
   {
      super.testStepOnAndOffCinderBlocks();
   }

   @Test
   @Override
   public void testStepOnCinderBlocks()
   {
      super.testStepOnCinderBlocks();
   }

   @Test
   @Override
   public void testStepOnCinderBlocksSlowlyWithDisturbance()
   {
      super.testStepOnCinderBlocksSlowlyWithDisturbance();
   }
}
