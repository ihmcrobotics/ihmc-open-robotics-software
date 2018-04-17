package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.HumanoidBehaviorDispatcherTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasBehaviorDispatcherTest extends HumanoidBehaviorDispatcherTest
{
   private final AtlasRobotModel robotModel;

   public AtlasBehaviorDispatcherTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
   }

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

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 63.1)
   @Test(timeout = 320000)
   public void testDispatchPelvisPoseBehavior() throws SimulationExceededMaximumTimeException
   {
      super.testDispatchPelvisPoseBehavior();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 315.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 1200000)
   public void testDispatchWalkToLocationBehavior() throws SimulationExceededMaximumTimeException
   {
      super.testDispatchWalkToLocationBehavior();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 57.8)
   @Test(timeout = 290000)
   public void testDispatchWalkToLocationBehaviorAndStop() throws SimulationExceededMaximumTimeException
   {
      super.testDispatchWalkToLocationBehaviorAndStop();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 102.0)
   @Test(timeout = 510000)
   public void testDispatchWalkToLocationBehaviorPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      super.testDispatchWalkToLocationBehaviorPauseAndResume();
   }
}
