package us.ihmc.valkyrie.obstacleCourse;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseRampsTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
public class ValkyrieObstacleCourseRampsTest extends DRCObstacleCourseRampsTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 100.6)
   @Test(timeout = 500000)
   public void testWalkingDownRampWithMediumSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingDownRampWithMediumSteps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 99.7)
   @Test(timeout = 500000)
   public void testWalkingUpRampWithMediumSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithMediumSteps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 500000)
   public void testWalkingUpRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithShortSteps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 104.2)
   @Test(timeout = 520000)
   public void testWalkingUpRampWithShortStepsALittleTooHigh() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithShortStepsALittleTooHigh();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 100.9)
   @Test(timeout = 500000)
   public void testWalkingUpRampWithShortStepsALittleTooLow() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithShortStepsALittleTooLow();
   }

   @Override
   protected double getMaxRotationCorruption()
   {
      return 0.0;
   }
}
