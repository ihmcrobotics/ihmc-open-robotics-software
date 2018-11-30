package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCFootstepListBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasFootstepListBehaviorTest extends DRCFootstepListBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasFootstepListBehaviorTest()
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
   @ContinuousIntegrationTest(estimatedDuration = 57.1)
   @Test(timeout = 290000)
   public void testSideStepping() throws SimulationExceededMaximumTimeException
   {
      super.testSideStepping();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 34.5)
   @Test(timeout = 170000)
   public void testStepLongerThanMaxStepLength() throws SimulationExceededMaximumTimeException
   {
      super.testStepLongerThanMaxStepLength();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 71.3)
   @Test(timeout = 360000)
   public void testStop() throws SimulationExceededMaximumTimeException
   {
      super.testStop();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 54.9)
   @Test(timeout = 270000)
   public void testTwoStepsForwards() throws SimulationExceededMaximumTimeException
   {
      super.testTwoStepsForwards();
   }
}
