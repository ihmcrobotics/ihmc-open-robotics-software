package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.RecoveringSwingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.RecoveryTransferState;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public class ContinuePushRecoveryWithNextStepCondition implements StateTransitionCondition
{
   private final RecoveryTransferState transferState;
   private final MultiStepPushRecoveryControlModule pushRecoveryControlModule;

   public ContinuePushRecoveryWithNextStepCondition(RecoveryTransferState transferState, MultiStepPushRecoveryControlModule pushRecoveryControlModule)
   {
      this.transferState = transferState;
      this.pushRecoveryControlModule = pushRecoveryControlModule;
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      if (!transferState.isDone(timeInState))
         return false;

      return pushRecoveryControlModule.isRobotFallingFromDoubleSupport() != null;
   }
}