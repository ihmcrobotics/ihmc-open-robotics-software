package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.DRCPushRecoveryWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasPushRecoveryWalkingTest extends DRCPushRecoveryWalkingTest
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
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushLeftEarlySwing();
   }

   // Moved one of the old push recovery tests to fast so it is checked from time to time.
   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushLeftInitialTransferState();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushRightInitialTransferState();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushRightLateSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushRightThenLeftMidSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushRightTransferState();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushTowardsTheBack();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushTowardsTheFront();
   }
}
