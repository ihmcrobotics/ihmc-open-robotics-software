package us.ihmc.atlas.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseTrialsWalkingTaskTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

@Tag("humanoid-obstacle-slow")
public class AtlasObstacleCourseTrialsWalkingTaskTest extends DRCObstacleCourseTrialsWalkingTaskTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
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
