package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseSteppingStonesTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

// moved to in development
// this test is flaky because toe off is not triggered properly - the toe off condition needs to be fixed
@ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT, IntegrationCategory.VIDEO})
public class AtlasObstacleCourseSteppingStonesTest extends DRCObstacleCourseSteppingStonesTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 48.9)
   @Test(timeout = 240000)
   public void testWalkingOverEasySteppingStones() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOverEasySteppingStones();
   }
}
