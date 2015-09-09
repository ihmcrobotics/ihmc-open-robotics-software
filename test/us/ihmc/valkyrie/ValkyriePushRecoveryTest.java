package us.ihmc.valkyrie;

import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.pushRecovery.DRCPushRecoveryTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = {TestPlanTarget.InDevelopment, TestPlanTarget.Slow})
public class ValkyriePushRecoveryTest extends DRCPushRecoveryTest
{
   protected DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 25.0)
   @Test(timeout = 163619)
   public void testControllerFailureKicksIn() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Fast);
      super.testControllerFailureKicksIn();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 26.0)
   @Test(timeout = 130000)
   public void testLongBackwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testLongBackwardPushWhileStanding();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 16.4)
   @Test(timeout = 82000)
   public void testLongBackwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testLongBackwardPushWhileStandingAfterControllerFailureKickedIn();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 15.7)
   @Test(timeout = 78000)
   public void testLongForwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testLongForwardPushWhileStanding();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 17.2)
   @Test(timeout = 86000)
   public void testLongForwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testLongForwardPushWhileStandingAfterControllerFailureKickedIn();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 35.7)
   @Test(timeout = 180000)
   public void testPushWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testPushWhileInSwing();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 22.2)
   @Test(timeout = 110000)
   public void testPushWhileInTransfer() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testPushWhileInTransfer();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 14.7)
   @Test(timeout = 73000)
   public void testPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testPushWhileStanding();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 14.4)
   @Test(timeout = 72000)
   public void testPushWhileStandingRecoveringAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testPushWhileStandingRecoveringAfterControllerFailureKickedIn();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 14.3)
   @Test(timeout = 71000)
   public void testRecoveringWithSwingSpeedUpWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testRecoveringWithSwingSpeedUpWhileInSwing();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 25.2)
   @Test(timeout = 130000)
   public void testRecoveryWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testRecoveryWhileInFlamingoStance();
   }
}
