package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCoursePlatformTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
public class AtlasObstacleCoursePlatformTest extends DRCObstacleCoursePlatformTest
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

   @Override
   // "This test is flaky. Sometimes it works, sometimes it doesn't due to threading of the various globalDataProducer and communicators. We need to be able to shut those off or make them not screw up the robot run.")
   @ContinuousIntegrationTest(estimatedDuration = 48.6, categoriesOverride = IntegrationCategory.FLAKY)
   @Test(timeout = 180000)
   public void testRunsTheSameWayTwiceJustStanding() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      super.testRunsTheSameWayTwiceJustStanding();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 67.5)
   @Test(timeout = 340000)
   public void testSidestepOverSmallPlatform() throws SimulationExceededMaximumTimeException
   {
      super.testSidestepOverSmallPlatform();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 65.5)
   @Test(timeout = 330000)
   public void testSidestepOverSmallWall() throws SimulationExceededMaximumTimeException
   {
      super.testSidestepOverSmallWall();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 38.5)
   @Test(timeout = 190000)
   public void testWalkingOffOfMediumPlatform() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOffOfMediumPlatform();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 58.5)
   @Test(timeout = 290000)
   public void testWalkingOffOfMediumPlatformSlowSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOffOfMediumPlatformSlowSteps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 39.0)
   @Test(timeout = 190000)
   public void testWalkingOntoMediumPlatformToesTouching() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOntoMediumPlatformToesTouching();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 59.6)
   @Test(timeout = 300000)
   public void testWalkingOverSmallPlatform() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOverSmallPlatform();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 39.4)
   @Test(timeout = 200000)
   public void testWalkingOverSmallPlatformQuickly() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOverSmallPlatformQuickly();
   }

}
