package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.DRCPushRecoveryTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasPushRecoveryTest extends DRCPushRecoveryTest
{
   @Override
   public double getAngledPushMagnitude()
   {
      return 350.0;
   }

   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testControllerFailureKicksIn() throws SimulationExceededMaximumTimeException
   {
      super.testControllerFailureKicksIn();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testLongBackwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testLongBackwardPushWhileStanding();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testLongBackwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testLongBackwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testLongForwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testLongForwardPushWhileStanding();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testLongForwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testLongForwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushICPOptimiWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimiWhileInSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileInSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushWhileInTransfer() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileInTransfer();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileStanding();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushWhileStandingRecoveringAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileStandingRecoveringAfterControllerFailureKickedIn();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoveringWithSwingSpeedUpWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveringWithSwingSpeedUpWhileInSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoveryWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveryWhileInFlamingoStance();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoveryForwardWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveryForwardWhileInFlamingoStance();
   }
}
