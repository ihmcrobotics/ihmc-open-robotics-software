package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseRampsTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
public class AtlasObstacleCourseRampsTest extends DRCObstacleCourseRampsTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

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
   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 500000)
   public void testWalkingDownRampWithMediumSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingDownRampWithMediumSteps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 96.1)
   @Test(timeout = 480000)
   public void testWalkingUpRampWithMediumSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithMediumSteps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 94.9)
   @Test(timeout = 470000)
   public void testWalkingUpRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithShortSteps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 94.9)
   @Test(timeout = 470000)
   public void testWalkingUpRampWithShortStepsALittleTooHigh() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithShortStepsALittleTooHigh();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 93.3)
   @Test(timeout = 470000)
   public void testWalkingUpRampWithShortStepsALittleTooLow() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithShortStepsALittleTooLow();
   }

   @Override
   protected double getMaxRotationCorruption()
   {
      return Math.PI/8.0;
   }

}
