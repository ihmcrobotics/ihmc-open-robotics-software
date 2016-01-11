package us.ihmc.valkyrie;

import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.pushRecovery.DRCPushRecoveryTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

public class ValkyriePushRecoveryTest extends DRCPushRecoveryTest
{
   protected DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 25.0, targets = TestPlanTarget.Fast)
   @Test(timeout = 163619)
   public void testControllerFailureKicksIn() throws SimulationExceededMaximumTimeException
   {
      super.testControllerFailureKicksIn();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 26.0, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 130000)
   public void testLongBackwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testLongBackwardPushWhileStanding();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 16.4, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 82000)
   public void testLongBackwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testLongBackwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 15.7, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 78000)
   public void testLongForwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testLongForwardPushWhileStanding();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 17.2, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 86000)
   public void testLongForwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testLongForwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 46.3, targets = TestPlanTarget.Slow)
   @Test(timeout = 230000)
   public void testPushWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileInSwing();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 22.2, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 110000)
   public void testPushWhileInTransfer() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileInTransfer();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 14.7, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 73000)
   public void testPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileStanding();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 14.4, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 72000)
   public void testPushWhileStandingRecoveringAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileStandingRecoveringAfterControllerFailureKickedIn();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 14.3, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 71000)
   public void testRecoveringWithSwingSpeedUpWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveringWithSwingSpeedUpWhileInSwing();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 25.2, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 130000)
   public void testRecoveryWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveryWhileInFlamingoStance();
   }
}
