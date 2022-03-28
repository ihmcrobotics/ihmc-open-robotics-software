package us.ihmc.valkyrie.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.pushRecovery.AvatarQuickPushRecoveryWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieSteppingParameters;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;

public class ValkyrieQuickPushRecoveryWalkingTest extends AvatarQuickPushRecoveryWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS)
      {
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new ValkyrieWalkingControllerParameters(getJointMap(), getRobotPhysicalProperties(), getTarget())
            {
               @Override
               public SteppingParameters getSteppingParameters()
               {
                  return new ValkyrieSteppingParameters(getRobotPhysicalProperties(), getTarget())
                  {
                     @Override
                     public double getMaxStepWidth()
                     {
                        return 0.8;
                     }
                  };

               };
            };
         }
      };
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testOutwardPushInitialTransferToLeftStateAndLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.5);
      super.testOutwardPushInitialTransferToLeftStateAndLeftMidSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testOutwardPushLeftSwingAtDifferentTimes() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.45);
      super.testOutwardPushLeftSwingAtDifferentTimes();
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
      setPushChangeInVelocity(0.5);
      super.testOutwardPushTransferToLeftState();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testBackwardPushInLeftSwingAtDifferentTimes() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.7);
      super.testBackwardPushInLeftSwingAtDifferentTimes();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testForwardPushInLeftSwing() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.7);
      super.testForwardPushInLeftSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testInwardPushLeftAtDifferentTimes() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.25);
      super.testInwardPushLeftAtDifferentTimes();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testForwardAndOutwardPushInLeftSwing() throws SimulationExceededMaximumTimeException
   {
      setPushChangeInVelocity(0.55);
      super.testForwardAndOutwardPushInLeftSwing();
   }
}