package us.ihmc.atlas;

import org.junit.Test;

import us.ihmc.avatar.DRCFlatGroundRewindabilityTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FLAKY)
public class AtlasFlatGroundRewindabilityTest extends DRCFlatGroundRewindabilityTest
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
   @ContinuousIntegrationTest(estimatedDuration = 9.2)
   @Test(timeout = 520000)
   public void testCanRewindAndGoForward() throws UnreasonableAccelerationException
   {
      super.testCanRewindAndGoForward();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 520000)
   public void testRewindabilityWithSimpleFastMethod() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      super.testRewindabilityWithSimpleFastMethod();
   }

   @Override
   // This takes a long time. Use it for debugging where the broken changes were made when the tests above fail.
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 520000)
   public void testRewindabilityWithSlowerMoreExtensiveMethod() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      super.testRewindabilityWithSlowerMoreExtensiveMethod();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 43.4)
   @Test(timeout = 520000)
   public void testRunsTheSameWayTwice() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException, ControllerFailureException
   {
      super.testRunsTheSameWayTwice();
   }

}
