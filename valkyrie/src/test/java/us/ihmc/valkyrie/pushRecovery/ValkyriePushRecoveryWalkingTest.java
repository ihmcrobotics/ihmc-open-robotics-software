package us.ihmc.valkyrie.pushRecovery;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.DRCPushRecoveryWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyriePushRecoveryWalkingTest extends DRCPushRecoveryWalkingTest
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

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(700.0);
      super.testPushLeftEarlySwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushLeftInitialTransferState();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightInitialTransferState();
   }

   /**
    * TODO: This test highlights an issue with the way the ICP optimization places the desired CMP in
    * case it is not achievable.
    * <p>
    * After the step adjustment the desired CMP should be moving to the heel to push the ICP into the
    * area of support after touchdown. Instead it is trying to achieve the current desired best as
    * possible causing a fall. This test used to pass with the old ICP control where the projection
    * method would switch to trying to push the ICP towards the final desired if the current desired
    * was not feasible.
    */
   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   @Disabled
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightLateSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(700.0);
      super.testPushRightThenLeftMidSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightTransferState();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Disabled
   @Test
   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException
   {
      super.testPushTowardsTheBack();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException
   {
      super.testPushTowardsTheFront();
   }
}