package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.RecoveryTransferState;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class RecoveryTransferToStandingCondition implements StateTransitionCondition
{
   private final RecoveryTransferState transferState;
   private final MultiStepPushRecoveryControlModule pushRecoveryControlModule;

   public RecoveryTransferToStandingCondition(RecoveryTransferState transferState, MultiStepPushRecoveryControlModule pushRecoveryControlModule)
   {
      this.transferState = transferState;
      this.pushRecoveryControlModule = pushRecoveryControlModule;
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      RobotSide swingSide = pushRecoveryControlModule.isRobotFallingFromDoubleSupport();

      // if falling and next support foot is not the transfer to side, go to standing to re-route state machine
      if (transferState.isICPPlanDone() && (swingSide != transferState.getTransferToSide().getOppositeSide()))
         return true;

      if (!transferState.isDone(timeInState))
         return false;

      return pushRecoveryControlModule.isRobotFallingFromDoubleSupport() == null;
   }
}