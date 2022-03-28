package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.pushRecovery.AvatarQuickPushRecoveryWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasQuickPushRecoveryWalkingTest extends AvatarQuickPushRecoveryWalkingTest
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

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testOutwardPushLeftEarlySwing() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.6);
      super.testOutwardPushLeftEarlySwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testInwardPushLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.3);
      super.testInwardPushLeftMidSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testOutwardPushInitialTransferToLeftStateAndLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.5);
      super.testOutwardPushInitialTransferToLeftStateAndLeftMidSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testOutwardPushMidLeftSwing() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.6);
      super.testOutwardPushMidLeftSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushOutwardInRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.5);
      super.testPushOutwardInRightThenLeftMidSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testOutwardPushTransferToLeftState() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.4);
      super.testOutwardPushTransferToLeftState();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testBackwardPushInLeftSwing() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.5);
      super.testBackwardPushInLeftSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testForwardPushInLeftSwing() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.4);
      super.testForwardPushInLeftSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testForwardAndOutwardPushInLeftSwing() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.8);
      super.testForwardAndOutwardPushInLeftSwing();
   }

}
