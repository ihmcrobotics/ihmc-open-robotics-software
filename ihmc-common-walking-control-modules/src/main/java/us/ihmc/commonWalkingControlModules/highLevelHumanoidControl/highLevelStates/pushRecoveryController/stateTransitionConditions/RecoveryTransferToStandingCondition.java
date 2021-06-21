package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.RecoveryTransferState;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class RecoveryTransferToStandingCondition implements StateTransitionCondition
{
   private final RecoveryTransferState transferState;
   private final MultiStepPushRecoveryController pushRecoveryControlModule;

   public RecoveryTransferToStandingCondition(RecoveryTransferState transferState, MultiStepPushRecoveryController pushRecoveryControlModule)
   {
      this.transferState = transferState;
      this.pushRecoveryControlModule = pushRecoveryControlModule;
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      if (!transferState.isDone(timeInState))
         return false;

      return !pushRecoveryControlModule.isRobotFallingFromDoubleSupport();
   }
}