package us.ihmc.valkyrie.pushRecovery;

import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.pushRecovery.DRCPushRecoveryTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@ContinuousIntegrationPlan(targets = TestPlanTarget.InDevelopment)
public class ValkyriePushRecoveryTest extends DRCPushRecoveryTest
{
   protected DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 16.3, targetsOverride = TestPlanTarget.Fast)
   @Test(timeout = 81000)
   public void testControllerFailureKicksIn() throws SimulationExceededMaximumTimeException
   {
      super.testControllerFailureKicksIn();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 26.0, targetsOverride = TestPlanTarget.InDevelopment)
   @Test(timeout = 130000)
   public void testLongBackwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testLongBackwardPushWhileStanding();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 16.4, targetsOverride = TestPlanTarget.InDevelopment)
   @Test(timeout = 82000)
   public void testLongBackwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testLongBackwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 15.7, targetsOverride = TestPlanTarget.InDevelopment)
   @Test(timeout = 78000)
   public void testLongForwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testLongForwardPushWhileStanding();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 17.2, targetsOverride = TestPlanTarget.InDevelopment)
   @Test(timeout = 86000)
   public void testLongForwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testLongForwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 46.3, targetsOverride = TestPlanTarget.InDevelopment)
   @Test(timeout = 230000)
   public void testPushWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 22.2, targetsOverride = TestPlanTarget.InDevelopment)
   @Test(timeout = 110000)
   public void testPushWhileInTransfer() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileInTransfer();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 14.7, targetsOverride = TestPlanTarget.InDevelopment)
   @Test(timeout = 73000)
   public void testPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileStanding();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 14.4, targetsOverride = TestPlanTarget.InDevelopment)
   @Test(timeout = 72000)
   public void testPushWhileStandingRecoveringAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileStandingRecoveringAfterControllerFailureKickedIn();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 14.3, targetsOverride = TestPlanTarget.InDevelopment)
   @Test(timeout = 71000)
   public void testRecoveringWithSwingSpeedUpWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveringWithSwingSpeedUpWhileInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 23.4, targetsOverride = TestPlanTarget.Fast)
   @Test(timeout = 120000)
   public void testRecoveryWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveryWhileInFlamingoStance();
   }
}
