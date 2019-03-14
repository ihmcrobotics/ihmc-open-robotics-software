package us.ihmc.valkyrie.pushRecovery;

import org.junit.jupiter.api.Disabled;
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

   @Override
   @Test
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(700.0);
      super.testPushLeftEarlySwing();
   }

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

   /**
    * TODO:
    * This test highlights an issue with the way the ICP optimization places the desired CMP in case it is not achievable.
    * <p>
    * After the step adjustment the desired CMP should be moving to the heel to push the ICP into the area of support after touchdown.
    * Instead it is trying to achieve the current desired best as possible causing a fall. This test used to pass with the old ICP control where the
    * projection method would switch to trying to push the ICP towards the final desired if the current desired was not feasible.
    */
   @Override
   @Test
   @Disabled
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightLateSwing();
   }

   @Override
   @Test
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(700.0);
      super.testPushRightThenLeftMidSwing();
   }

   @Override
   @Test
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightTransferState();
   }

   @Override
   @Disabled
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