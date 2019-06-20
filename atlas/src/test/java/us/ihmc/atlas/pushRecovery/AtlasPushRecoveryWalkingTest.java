package us.ihmc.atlas.pushRecovery;

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

   @Override
   @Test
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(700.0);
      super.testPushLeftEarlySwing();
   }

   // Moved one of the old push recovery tests to fast so it is checked from time to time.
   @Override
   @Test
   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushLeftInitialTransferState();
   }

   @Override
   @Test
   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightInitialTransferState();
   }

   @Override
   @Test
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightLateSwing();
   }

   @Override
   @Test
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(800.0);
      super.testPushRightThenLeftMidSwing();
   }

   @Override
   @Test
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightTransferState();
   }

   @Override
   @Test
   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException
   {
      super.testPushTowardsTheBack();
   }

   @Override
   @Test
   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException
   {
      super.testPushTowardsTheFront();
   }
}
