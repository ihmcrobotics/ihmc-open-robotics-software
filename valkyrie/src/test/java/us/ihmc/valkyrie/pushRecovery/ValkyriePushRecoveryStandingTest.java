package us.ihmc.valkyrie.pushRecovery;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.avatar.DRCPushRecoveryStandingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class ValkyriePushRecoveryStandingTest extends DRCPushRecoveryStandingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Ignore("Needs to be improved")
   @ContinuousIntegrationTest(estimatedDuration = 53.2)
   @Test(timeout = 270000)
   @Override
   public void testDoublePushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testDoublePushForwardInDoubleSupportAndContinueWalking();
   }

   @ContinuousIntegrationTest(estimatedDuration = 80.2)
   @Test(timeout = 270000)
   @Override
   public void testPushBackwardForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushBackwardForwardInDoubleSupportAndContinueWalking();
   }

   @ContinuousIntegrationTest(estimatedDuration = 70.2)
   @Test(timeout = 270000)
   @Override
   public void testPushBackwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushBackwardInDoubleSupportAndContinueWalking();
   }

   @ContinuousIntegrationTest(estimatedDuration = 53.2)
   @Test(timeout = 270000)
   @Override
   public void testPushForwardInDoubleSupport() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushForwardInDoubleSupport();
   }

   @ContinuousIntegrationTest(estimatedDuration = 75.0)
   @Test(timeout = 270000)
   @Override
   public void testPushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushForwardInDoubleSupportAndContinueWalking();
   }

}
