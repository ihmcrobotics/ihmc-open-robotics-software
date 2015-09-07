package us.ihmc.valkyrie;

import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.pushRecovery.DRCPushRecoveryTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

@BambooPlan(planType = {BambooPlanType.InDevelopment, BambooPlanType.Slow})
public class ValkyriePushRecoveryTest extends DRCPushRecoveryTest
{
   protected DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
   }
   
   @Override
   @EstimatedDuration(duration = 25.0)
   @Test(timeout = 163619)
   public void testControllerFailureKicksIn() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Fast);
      super.testControllerFailureKicksIn();
   }
   
   @Override
	@EstimatedDuration(duration = 26.0)
   @Test(timeout = 130000)
   public void testLongBackwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testLongBackwardPushWhileStanding();
   }
   
   @Override
	@EstimatedDuration(duration = 16.4)
   @Test(timeout = 82000)
   public void testLongBackwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testLongBackwardPushWhileStandingAfterControllerFailureKickedIn();
   }
   
   @Override
	@EstimatedDuration(duration = 15.7)
   @Test(timeout = 78000)
   public void testLongForwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testLongForwardPushWhileStanding();
   }
   
   @Override
	@EstimatedDuration(duration = 17.2)
   @Test(timeout = 86000)
   public void testLongForwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testLongForwardPushWhileStandingAfterControllerFailureKickedIn();
   }
   
   @Override
	@EstimatedDuration(duration = 35.7)
   @Test(timeout = 180000)
   public void testPushWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testPushWhileInSwing();
   }
   
   @Override
	@EstimatedDuration(duration = 22.2)
   @Test(timeout = 110000)
   public void testPushWhileInTransfer() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testPushWhileInTransfer();
   }
   
   @Override
	@EstimatedDuration(duration = 14.7)
   @Test(timeout = 73000)
   public void testPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testPushWhileStanding();
   }
   
   @Override
	@EstimatedDuration(duration = 14.4)
   @Test(timeout = 72000)
   public void testPushWhileStandingRecoveringAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testPushWhileStandingRecoveringAfterControllerFailureKickedIn();
   }
   
   @Override
	@EstimatedDuration(duration = 14.3)
   @Test(timeout = 71000)
   public void testRecoveringWithSwingSpeedUpWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testRecoveringWithSwingSpeedUpWhileInSwing();
   }
   
   @Override
	@EstimatedDuration(duration = 25.2)
   @Test(timeout = 130000)
   public void testRecoveryWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testRecoveryWhileInFlamingoStance();
   }
}
